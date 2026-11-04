/*
 * 40MHz SPI 从机监听引擎（Core 1）
 *
 * 数据通路：
 *
 *   CS/SCK/MOSI/MISO ──> PIO 状态机 ──> RX FIFO ──> DMA ──> 32KB 环形缓冲
 *                          200MHz      8 级深      ENDLESS     自动回绕
 *                          每周期采 4 根线，8 个采样点 40ns 组成一个 32bit 字，
 *                          即每秒 2500 万字，环形缓冲覆盖约 328µs 历史。
 *
 *   采集是"自由运行、永不停止"的：不按帧启停状态机，所以不存在两个状态
 *   之间的重新武装窗口，也就不会漏掉帧头。帧的起止完全由软件在采样流里
 *   找 CS 的电平变化决定。
 *
 *   Core 1 的工作循环：
 *     1. CS 下降沿中断记录"一帧开始了"（主循环在忙也不会漏）；
 *     2. 等 CS 连续回高 5µs（一帧结束），或超时兜底；
 *     3. 回头在环形缓冲里，从这个 CS 下降沿开始把帧解出来；
 *     4. 结果写进全局 g_sniff，Core 0 负责显示和通过串口输出。
 *
 * 强制约束：CS、SCK、MOSI、MISO 必须依次接在连续的 4 个 GPIO 上，
 * 因为 PIO 用一条 `in pins,4` 同时锁存四根线。
 */

#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "spi_sniffer.pio.h"
#include "board.h"
#include "sniffer.h"

/* 编译期检查：四根线必须连续。若在 board.h 里改了引脚却不符合这个要求，
 * 直接编译失败，比烧到板子上再发现"数据全错"要好得多。 */
#if (PIN_SNIFF_SCK != (PIN_SNIFF_CS + 1)) || \
    (PIN_SNIFF_MOSI != (PIN_SNIFF_SCK + 1)) || \
    (PIN_SNIFF_MISO != (PIN_SNIFF_MOSI + 1))
#error "过采样要求 CS/SCK/MOSI/MISO 依次接在连续的 4 个 GPIO 上"
#endif
/* PIO 的 in pins 指令只能寻址 GPIO0~GPIO31，起始脚 + 4 不能越界 */
_Static_assert(PIN_SNIFF_CS + 4 <= 32, "PIO 的 in pins 只能访问 GPIO0~GPIO31");

sniff_shared_t g_sniff;

/*
 * 采样环形缓冲。
 *
 * 大小 32KB 是 RP2350 DMA 写地址回绕（write-address ring）的上限：
 * 回绕范围由 2^15 = 32768 字节决定，所以缓冲必须按 32KB 对齐声明。
 * 32768 字节 = 8192 个字 = 65536 个采样点，按 200MHz 采样即 328µs 历史。
 *
 * 声明成 volatile：这块内存由 DMA 异步改写，编译器不能把它缓存进寄存器，
 * 也不能为了"优化"而省略某次读取。
 */
#define RING_WORDS      8192u
#define RING_BYTES      (RING_WORDS * 4u)
#define RING_SAMPLES    (RING_WORDS * 8u)          /* 65536 个采样点 = 328µs */
#define RING_SMASK      (RING_SAMPLES - 1u)        /* 采样位置取模用的掩码 */

static volatile uint32_t s_ring[RING_WORDS] __attribute__((aligned(RING_BYTES)));

/* 往回找 CS 下降沿的最大距离：1/4 个环 = 16384 个采样点 = 82µs。
 * 当前 64 字节帧只有 14µs，足够覆盖；窗口再大意义不大，反而增加扫描时间。 */
#define MAX_BACK_SAMPLES   (RING_SAMPLES / 4u)

/* 单帧最多解析 100µs（20000 个采样点）：40MHz 下就是 4000 bit，
 * 远超实际帧长。超过就按"被截断"处理，防止 CS 卡低时无限扫描。 */
#define MAX_FRAME_SAMPLES  20000u

/* CS 一直不回高（线没接好、被测设备卡死等）时的兜底超时 */
#define FRAME_TIMEOUT_US  50000u

/* 帧结束判定：CS 必须连续为高这么久才算这一帧真的结束。
 * 只看单次上升沿会把 CS 边沿的振铃当成帧尾——振铃造成的一两个采样点
 * 假边沿会被解成"0 bit 的帧"，既没用又会打断正在进行的真帧。 */
#define FRAME_END_US      5u

/* 帧起点判定：看下降沿之后的 CS_STABLE_WINDOW 个采样点（160ns），
 * 其中至少 CS_STABLE_MIN 个为低才算真帧。
 * 用"比例"而不是"连续"是因为 CS 边沿本身可能有几十 ns 的振铃：
 * 振铃只会让窗口里少几个低电平点，不会把整段低电平抹掉；
 * 而振铃造成的假下降沿后面几乎全是高电平，比例远低于门槛。 */
#define CS_STABLE_WINDOW 32u
#define CS_STABLE_MIN    20u

/* 帧结束判定：CS 为高，并且连续这么多个采样点内没有出现过 SCK 上升沿。
 *
 * 只看 "CS 连续为高" 是不够的。40MHz 的长连线上，SCK/MOSI/MISO 的快速
 * 翻转会通过容性串扰/地弹耦合到 CS：实测原始采样里帧中间就混有 1~2 点
 * 的假高电平（例：`in pins,4` 采到 0xB/0x7 这种一个 nibble 里三根线同时
 * 为高的点），长帧中途还出现过 ≥8 点（40ns）的假高，旧判据因此在第
 * 284 bit 处把 13.8µs 的 64 字节读帧截成了 35 字节。
 *
 * 但 SPI 事务的本质是时钟：只要 SCK 还在翻转，事务就没结束；真正结束
 * 时主机会停住时钟。所以判据改成 "CS 高 + 320ns 无 SCK 上升沿"——
 * 时钟在走的时候任何 CS 毛刺都不会结束帧，也就不受毛刺长度影响。
 * 200MHz 下 64 点 = 320ns，远小于两次事务之间的空闲（实测约 4µs），
 * 不会把相邻两帧粘在一起。 */
#define CS_IDLE_END_SAMPLES 64u

/*
 * 采样点里四个字段的取法。
 *
 * PIO 的字格式（见 spi_sniffer.pio）：
 *   bit[4k+0] = CS   bit[4k+1] = SCK   bit[4k+2] = MOSI   bit[4k+3] = MISO
 *   k = 0 是该字里最早的采样点（右移入 ISR，最早的点落在最低 4 位）。
 */
#define S_CS(s)    ((s) & 1u)
#define S_SCK(s)   (((s) >> 1) & 1u)
#define S_MOSI(s)  (((s) >> 2) & 1u)
#define S_MISO(s)  (((s) >> 3) & 1u)

/* DMA 通道号，-1 表示还没申请。decode_last_frame 要靠它读 DMA 写指针 */
static int s_dma_chan = -1;

/* 本帧解出的完整字节流（Core 1 私有）。发布到共享区时才拷贝，
 * 这样被过滤的短帧不会破坏上一帧已发布的数据。 */
static uint8_t s_dump_mosi[SNIFF_DUMP_BYTES];
static uint8_t s_dump_miso[SNIFF_DUMP_BYTES];
static uint32_t s_dump_bytes;

/* 原始采样流转储次数：解码失败最多抓 2 次，长帧单独抓 1 次。
 * 两者必须分开计数，否则开机时的两次失败会把唯一的长帧转储名额用掉，
 * 真正要看的 64 字节读帧反而没有原始波形。 */
static uint32_t s_raw_fail_dumps;
static uint32_t s_raw_long_dumps;
static uint32_t s_raw_seq;

/* CS 下降沿中断：Core 1 进入解析后，轮询引脚是"瞎"的（解析一帧要几十
 * 微秒，而 14µs 的 64 字节读帧可能在这段时间里开始并结束）。因此改用
 * 下降沿中断把"有新帧"这件事先记下来，主循环再从容地去环形缓冲里取。
 *
 * 中断只做两件事：置标志、记下发生时间。环形缓冲覆盖 328µs，只要解析
 * 一帧的耗时短于这个窗口，就不会丢帧。 */
static volatile bool     s_cs_event;
static volatile uint64_t s_cs_fall_us;

static void cs_fall_isr(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    s_cs_fall_us = time_us_64();
    s_cs_event = true;
}

/* 诊断快照写入序号（Core 1 私有） */
static uint32_t s_diag_seq;

/* 最后一次"发布"出去的结果状态。屏幕上的 STATE 显示它，而不是当前这一帧
 * 的瞬时状态：命令帧被过滤后要回到这里，否则界面会停在 CAPTURING。 */
static uint32_t s_published_state = SNIFF_ARMED;

/*
 * 取环形缓冲里某个绝对位置（0 .. RING_SAMPLES-1）的 4bit 采样点。
 *
 * pos 是"采样点序号"：pos >> 3 得到第几个 32bit 字（每字 8 个采样点），
 * pos & 7 得到字内第几个采样点，再右移 4*(k) 位取出对应的 nibble。
 *
 * 注意：采样位置会自然回绕，调用方在传递 pos 时已经用 RING_SMASK 取过模。
 */
static inline uint32_t sample_at(uint32_t pos)
{
    uint32_t w = s_ring[pos >> 3];
    return (w >> (4u * (pos & 7u))) & 0xFu;
}

/*
 * 一帧的解析结果（Core 1 内部使用，随后由 publish_frame 复制到共享区）。
 */
typedef struct {
    uint32_t start;         /* 帧起点在环形缓冲里的绝对采样位置（取模后） */
    uint32_t bits;          /* 帧内 SCK 上升沿个数，也就是每条线的 bit 数 */
    uint32_t samples;       /* 帧内采样点个数（1 个点 = 5ns） */
    uint32_t hi_min;        /* SCK 高电平最短宽度（采样点数，5ns/点） */
    uint32_t hi_max;        /* SCK 高电平最长宽度（采样点数，5ns/点） */
    uint32_t hash_mosi;     /* 整帧 MOSI 的 FNV-1a 哈希 */
    uint32_t hash_miso;     /* 整帧 MISO 的 FNV-1a 哈希 */
    uint32_t first_count;   /* first_xxx 里实际有效的字节数（最多 8） */
    uint8_t first_mosi[8];  /* 前 8 个 MOSI 字节，供屏幕直接显示 */
    uint8_t first_miso[8];  /* 前 8 个 MISO 字节 */

    /* ---- 帧尾判定与起点扫描的证据（诊断用）---- */
    uint32_t end_run;       /* 帧尾时 CS 连续为高的点数；0 = 撞写指针/到上限 */
    uint32_t hit_head;      /* 1 = 解析追到 DMA 写指针，帧尾还没写进环 */
    uint32_t cand;          /* 回溯扫描遇到的 CS 下降沿候选数 */
    uint32_t max_low;       /* 候选中下降沿之后 32 点里 CS 低的最大个数 */
    uint32_t max_high;      /* 候选中下降沿之前 32 点里 CS 高的最大个数 */
} frame_result_t;

/*
 * 从环形缓冲里解出最近一帧。
 *
 * 参数：
 *   r —— 输出参数。函数会先用 0 填满，再把解出的字段写进去。
 *
 * 返回值：
 *   true  —— 在缓冲里找到了一个稳定的 CS 下降沿，并至少解析到了帧尾
 *            （或达到解析上限被截断）；
 *   false —— 最近 MAX_BACK_SAMPLES（82µs）内没有稳定的 CS 下降沿，
 *            本次没有可解析的帧。
 *
 * 副作用：
 *   - 填写 g_sniff 里的环形缓冲诊断字段（ring_cs_hi / ring_tail /
 *     ring_head / dma_waddr）；
 *   - 把解出的字节流写进 s_dump_mosi / s_dump_miso，供发布时使用。
 *
 * 解析步骤：
 *   1) 取 DMA 当前写指针，换算出"最新采样点"的位置 head；
 *   2) 从 head 往回扫，找最近一次 CS 由高变低，并用其后 32 个点验证稳定性；
 *   3) 从帧起点往后扫，直到 "CS 为高且 320ns 没有 SCK 上升沿"（帧结束）；
 *   4) 扫描过程中，每个 SCK 上升沿取一次 MOSI/MISO，拼成字节；
 *      同时统计 SCK 高电平宽度。SPI Mode 0 下数据在上升沿有效。
 */
static bool decode_last_frame(frame_result_t *r)
{
    memset(r, 0, sizeof(*r));
    r->hi_min = 0xFFFFFFFFu;      /* 还没量到高电平时用一个"很大"的哨兵值 */
    r->hash_mosi = 2166136261u;   /* FNV-1a 初始值 */
    r->hash_miso = 2166136261u;
    s_dump_bytes = 0;

    /* ---- 1) 取 DMA 写指针，换算成环内采样位置 ----
     * write_addr 是"下一个将要写入的地址"，所以最新的完整采样点
     * 在 head-1 处。若地址不在环内（理论上不会），把诊断字段置成
     * 明显的异常值并放弃本次解析。 */
    uint32_t waddr = (uint32_t)(uintptr_t)dma_channel_hw_addr(s_dma_chan)->write_addr;
    uint32_t base = (uint32_t)(uintptr_t)s_ring;
    uint32_t wi = (waddr - base) >> 2;
    g_sniff.dma_waddr = waddr;
    if (wi >= RING_WORDS) {
        g_sniff.ring_cs_hi = 0xFFFFFFFFu;
        g_sniff.ring_tail = 0xFFFFFFFFu;
        return false;
    }
    uint32_t head = wi << 3;      /* 下一个将被写入的采样点位置 */
    g_sniff.ring_head = head;

    /* ---- 2) 环形缓冲健康检查（诊断用）----
     * ring_cs_hi：最近 4096 个采样点里 CS 为高的个数。空闲时应接近 4096，
     *             为 0 说明 DMA 没在写，或 PIO 采到的 CS 恒低。
     *             CS 是每个 nibble 的 bit0，一个字里 8 个点对应
     *             0x11111111 掩码；按字取掩码再 popcount，512 个字就能
     *             统计完 4096 个点，比逐点取快 8 倍左右。这是解析前的
     *             诊断，不能拖慢主循环，否则又会漏掉后面的帧。
     * ring_tail ：最近 8 个采样点的原始 nibble，拼成一个 32bit 值。
     *             空闲时每个 nibble 应是 1（CS=1）或 9（MISO 有上拉时）。 */
    {
        uint32_t wi_head = head >> 3;      /* 下一个要写的字 */
        uint32_t hi = 0;
        for (uint32_t i = 0; i < 512u; i++) {
            uint32_t w = s_ring[(wi_head - 1u - i) & (RING_WORDS - 1u)];
            hi += (uint32_t)__builtin_popcount(w & 0x11111111u);
        }
        g_sniff.ring_cs_hi = hi;

        uint32_t tail = 0;
        for (uint32_t i = 0; i < 8u; i++) {
            tail = (tail << 4) | sample_at((head - 8u + i) & RING_SMASK);
        }
        g_sniff.ring_tail = tail;
    }

    /* ---- 3) 往回找 CS 下降沿 ----
     * 从最新点向旧的方向扫。`newer` 是后一个（较新）采样点的 CS，
     * `older` 是前一个（较旧）的点；older=1、newer=0 就说明两者之间
     * 发生了 CS 高->低。
     *
     * 找到候选后，前后各看 32 个点：
     *   - 下降沿之后至少 20 个低电平：真帧的 CS 会一直保持低；
     *   - 下降沿之前至少 20 个高电平：真帧之前总线是空闲的。
     * 第二个条件专门用来排除"帧中间 CS 的短毛刺"：毛刺也有一个下降沿，
     * 但它前面还是帧内的低电平，所以不满足；而真帧起点之前是长时间空闲
     * 的高电平。少了这个判据，16 bit 的状态帧会被从毛刺处切开，解成
     * 8 bit——日志里那些 lbits=8 而实际应该是 16 的帧就是这么来的。 */
    uint32_t newer = S_CS(sample_at((head - 1u) & RING_SMASK));
    uint32_t start = 0;
    bool found = false;

    for (uint32_t j = 2; j <= MAX_BACK_SAMPLES; j++) {
        uint32_t pos = (head - j) & RING_SMASK;
        uint32_t older = S_CS(sample_at(pos));
        if (older && !newer) {
            uint32_t low = 0;
            uint32_t high = 0;
            for (uint32_t k = 1; k <= CS_STABLE_WINDOW; k++) {
                if (!S_CS(sample_at((pos + k) & RING_SMASK))) {
                    low++;
                }
                if (S_CS(sample_at((pos - k) & RING_SMASK))) {
                    high++;
                }
            }
            /* 记录候选边的质量：解码失败时，这两个数能说明是“起点之后
             * 被 CS 毛刺打断”（low 偏小）还是“起点之前总线没空闲”
             * （high 偏小），比事后猜波形可靠。 */
            r->cand++;
            if (low > r->max_low) {
                r->max_low = low;
            }
            if (high > r->max_high) {
                r->max_high = high;
            }
            if (low >= CS_STABLE_MIN && high >= CS_STABLE_MIN) {
                start = (pos + 1u) & RING_SMASK;   /* 帧从下降沿之后开始 */
                found = true;
                break;
            }
        }
        newer = older;
    }
    if (!found) {
        return false;             /* 最近 82µs 内没有稳定的 CS 下降沿 */
    }
    r->start = start;             /* 供主循环去重：同一帧不重复处理 */

    /* ---- 4) 正向解析整帧 ---- */
    /* 边沿检测需要一个"前一个 SCK 电平"作为初值：取帧起点前一个采样点。
     * SPI Mode 0 下这个点通常是低。 */
    uint32_t prev_sck = S_SCK(sample_at((start - 1u) & RING_SMASK));
    uint32_t hi_run = 0;          /* 当前连续高电平的长度（采样点数） */
    uint32_t cs_high_run = 0;     /* 当前连续 CS 高电平的长度，用于判帧尾 */
    uint32_t idle_run = 0;        /* CS 为高且没有 SCK 上升沿的连续点数 */
    uint32_t nbit = 0;            /* 当前字节已拼了几位 */
    uint8_t mosi_byte = 0;
    uint8_t miso_byte = 0;

    for (uint32_t n = 0; n < MAX_FRAME_SAMPLES; n++) {
        uint32_t pos = (start + n) & RING_SMASK;
        if (pos == head) {
            /* 追到 DMA 写指针了：这一帧还没写完，后面是上一圈留下的旧数据，
             * 不能再往后读。按"被截断"收尾，避免把旧采样当成这一帧的后半段。
             * hit_head 会让上层把这一帧标成不完整，便于和 CS 毛刺区分。 */
            r->samples = n;
            r->hit_head = 1;
            return true;
        }
        uint32_t s = sample_at(pos);
        uint32_t cs = S_CS(s);
        uint32_t sck = S_SCK(s);
        bool sck_rise = (sck && !prev_sck);

        /* 帧尾判定：CS 为高，并且连续 CS_IDLE_END_SAMPLES 个点内没有 SCK
         * 上升沿。时钟还在走就说明事务没结束——这样任何长度的 CS 串扰
         * 毛刺都不会把长帧从中间截断（见 CS_IDLE_END_SAMPLES 的说明）。
         * idle_run 只在 CS 为高时累加，CS 一低或来一个时钟沿就清零。
         *
         * samples 取"CS 第一次变高"的位置，而不是检测到结束时的位置，
         * 这样 320ns 的确认时间和之后的空闲都不会算进帧长。 */
        if (cs) {
            cs_high_run++;
            idle_run = sck_rise ? 0u : idle_run + 1u;
            if (idle_run >= CS_IDLE_END_SAMPLES) {
                /* 已经确认帧尾；再往后最多看 256 个点，量出 CS 实际保持
                 * 高电平的长度（跨帧间隔）。诊断用：万一以后长帧又被截断，
                 * 这个值能区分"真的到帧尾了"和"遇到了超长毛刺"。 */
                uint32_t first_hi = n + 1u - cs_high_run;
                uint32_t run = cs_high_run;
                while (run < 256u) {
                    uint32_t p2 = (start + first_hi + run) & RING_SMASK;
                    if (p2 == head || !S_CS(sample_at(p2))) {
                        break;
                    }
                    run++;
                }
                r->end_run = run;
                r->samples = first_hi;
                return true;
            }
        } else {
            cs_high_run = 0;
            idle_run = 0;
        }

        /* 注意：CS 短暂为高时不能跳过下面的时钟解析。
         * 毛刺期间 SCK 仍在翻转，若跳过这几个点，正好落在毛刺上的一个
         * SCK 上升沿就会丢——离线模拟里 16 bit 的状态帧被解成 15 bit
         * 就是这个原因。真正的帧尾 SCK 已经空闲为低，不会多计位。 */

        /* 高电平宽度统计：一段完整的高电平结束时记一次。
         * 单位是采样点（5ns），所以 40MHz、50% 占空比时应为 2~3。 */
        if (sck) {
            hi_run++;
        } else if (hi_run) {
            if (hi_run < r->hi_min) {
                r->hi_min = hi_run;
            }
            if (hi_run > r->hi_max) {
                r->hi_max = hi_run;
            }
            hi_run = 0;
        }

        /* SCK 上升沿（当前高、上一个点低）就是一位数据有效的位置。
         * SPI 是 MSB 先出：每来一位左移一位再补到最低位。 */
        if (sck_rise) {
            mosi_byte = (uint8_t)((mosi_byte << 1) | S_MOSI(s));
            miso_byte = (uint8_t)((miso_byte << 1) | S_MISO(s));

            if (++nbit == 8) {
                /* 前 8 个字节留给屏幕显示 */
                if (r->first_count < 8) {
                    r->first_mosi[r->first_count] = mosi_byte;
                    r->first_miso[r->first_count] = miso_byte;
                    r->first_count++;
                }
                /* 完整字节流留给串口核对。帧长超过 SNIFF_DUMP_BYTES 时只
                 * 截断转储，不影响下面的位计数与哈希——那样仍能判断
                 * "采了多少 bit"。 */
                if (s_dump_bytes < SNIFF_DUMP_BYTES) {
                    s_dump_mosi[s_dump_bytes] = mosi_byte;
                    s_dump_miso[s_dump_bytes] = miso_byte;
                    s_dump_bytes++;
                }
                /* FNV-1a：同一地址连读时，每帧哈希应完全一致 */
                r->hash_mosi = (r->hash_mosi ^ mosi_byte) * 16777619u;
                r->hash_miso = (r->hash_miso ^ miso_byte) * 16777619u;
                nbit = 0;
            }
            r->bits++;
        }

        prev_sck = sck;
        r->samples = n + 1;       /* 若最后没走到帧尾分支，这里就是已扫长度 */
    }

    /* 跑到解析上限：CS 还没回高，按截断处理（返回 true，但 frame_full 会置位） */
    return true;
}

/*
 * 把一帧结果发布到共享区 g_sniff，供 Core 0 显示/打印。
 *
 * 参数：
 *   r          —— 解析结果；
 *   elapsed_us —— CS 下降到 CS 回高的时间（µs），用于估算 SCK 频率；
 *   ok         —— 帧是否正常结束（CS 回高）；
 *   full       —— 是否因为达到解析上限被截断；
 *   frame      —— 本帧的发布序号（从 1 开始）。
 *
 * 发布顺序很重要：
 *   先写完所有数据字段，再把 frame_no 放到最后写（中间有 __dmb()）。
 *   Core 0 以 frame_no 变化作为"有新帧"的信号，这个顺序保证它看到新帧号
 *   时，cap_mosi/cap_miso 等数据一定已经写完整，不会读到上一帧的残留。
 */
static void publish_frame(const frame_result_t *r, uint32_t elapsed_us,
                          bool ok, bool full, uint32_t frame)
{
    for (uint32_t i = 0; i < 8; i++) {
        g_sniff.first_mosi[i] = (i < r->first_count) ? r->first_mosi[i] : 0u;
        g_sniff.first_miso[i] = (i < r->first_count) ? r->first_miso[i] : 0u;
    }

    g_sniff.hash_mosi = r->hash_mosi;
    g_sniff.hash_miso = r->hash_miso;
    g_sniff.frame_bits = r->bits;
    g_sniff.bytes_line = r->bits / 8u;
    g_sniff.samples = r->samples;
    g_sniff.sck_hi_min = (r->hi_min == 0xFFFFFFFFu) ? 0u : r->hi_min;
    g_sniff.sck_hi_max = r->hi_max;
    g_sniff.frame_ok = ok ? 1u : 0u;
    g_sniff.frame_full = full ? 1u : 0u;
    g_sniff.elapsed_us = elapsed_us;
    g_sniff.sck_hz = elapsed_us
        ? (uint32_t)(((uint64_t)r->bits * 1000000u) / elapsed_us)
        : 0u;                     /* bit 数 / 帧时长 = 估算的 SCK 频率 */

    /* 拷贝字节流供串口核对 */
    uint32_t n = (s_dump_bytes < SNIFF_DUMP_BYTES) ? s_dump_bytes : SNIFF_DUMP_BYTES;
    for (uint32_t i = 0; i < n; i++) {
        g_sniff.cap_mosi[i] = s_dump_mosi[i];
        g_sniff.cap_miso[i] = s_dump_miso[i];
    }
    g_sniff.cap_bytes = n;

    /* 标志字段最后写：Core 0 看到新帧号时，上面的数据一定已就绪 */
    __dmb();
    g_sniff.frame_no = frame;
}

/*
 * 用 XOSC 时基实测 clk_sys（单位 kHz）。
 *
 * 原理：PIO 的 clk_cal 程序从 TX FIFO 取一个计数值 n，在 clkdiv=1 下执行
 * 一个 1 周期的空循环 n 次，完成后 push 一个标志。Core 1 用 time_us_64()
 * 量这段耗时。time_us_64() 走 TIMER 外设，时钟源是 12MHz 晶振，与 clk_sys
 * 无关，所以结果是真正的频率测量，而不是读 PLL 配置寄存器自报的值。
 *
 *   实测频率 = n 个周期 / 实测时间
 *
 * 参数 pio：使用哪一个 PIO 实例（本工程是 pio0）。
 * 返回：kHz；测量失败（耗时为 0）时返回 0。
 *
 * 注意：临时占用的状态机会归还（unclaim），但 clk_cal 程序本身仍留在
 * PIO 指令存储里，这是有意为之——pio_remove_program 会让后续程序的偏移
 * 变化，没必要为了省几条指令去动它。
 */
static uint32_t measure_clk_sys_khz(PIO pio)
{
    const uint32_t n = 50000000u;      /* 200MHz 下 0.25s，150MHz 下 0.33s */

    uint cal_sm = pio_claim_unused_sm(pio, true);
    uint cal_off = pio_add_program(pio, &clk_cal_program);
    pio_sm_config cal_cfg = clk_cal_program_get_default_config(cal_off);
    pio_sm_init(pio, cal_sm, cal_off, &cal_cfg);

    /* 先把计数值放进 TX FIFO，再开表：这样计时起点和循环起点对齐，
     * 不会把"等待 FIFO 数据"的时间算进测量结果。 */
    pio_sm_put_blocking(pio, cal_sm, n);
    uint64_t t0 = time_us_64();
    pio_sm_set_enabled(pio, cal_sm, true);
    (void)pio_sm_get_blocking(pio, cal_sm);   /* 阻塞到程序 push 出完成标志 */
    uint64_t dt_us = time_us_64() - t0;

    pio_sm_set_enabled(pio, cal_sm, false);
    pio_sm_unclaim(pio, cal_sm);

    if (dt_us == 0) {
        return 0;
    }
    return (uint32_t)(((uint64_t)n * 1000u) / dt_us);   /* kHz */
}

/*
 * Core 1 入口。由 Core 0 调用 multicore_launch_core1() 启动，永不返回。
 *
 * 初始化顺序（每一步失败都会在屏幕上体现，便于定位）：
 *   1. 清零共享区，记录 clk_sys 配置值；
 *   2. 四个监听脚配成高阻输入（绝不影响被测总线）；
 *   3. 等总线空闲（CS 为高）再开始；
 *   4. 配置 PIO：一条 `in pins,4`，clkdiv=1，右移 + 满 32bit 自动 push；
 *   5. 检查程序确实是那一条指令（防止 .pio 被改坏）；
 *   6. 实测 clk_sys（约 0.25s）；
 *   7. 配置 DMA：PIO RX FIFO -> 32KB 环形缓冲，ENDLESS 永不停止；
 *   8. 使能状态机，进入"等 CS 帧 -> 解析 -> 发布"的主循环。
 */
void sniffer_core1_entry(void)
{
    memset(&g_sniff, 0, sizeof(g_sniff));
    g_sniff.clk_sys_hz = clock_get_hz(clk_sys);

    /* ---- 2) 监听脚一律高阻输入 ----
     * gpio_init 会把引脚交给 SIO、设为输入并使能输入缓冲；
     * gpio_disable_pulls 关闭内部上下拉，避免我们反过来影响被测总线。
     * 注意：输入功能在 RP2350 上还要求 pad 的 isolation 被解除，
     * gpio_init 已经处理了这一点。 */
    gpio_init(PIN_SNIFF_CS);
    gpio_init(PIN_SNIFF_SCK);
    gpio_init(PIN_SNIFF_MOSI);
    gpio_init(PIN_SNIFF_MISO);
    gpio_set_dir(PIN_SNIFF_CS, GPIO_IN);
    gpio_set_dir(PIN_SNIFF_SCK, GPIO_IN);
    gpio_set_dir(PIN_SNIFF_MOSI, GPIO_IN);
    gpio_set_dir(PIN_SNIFF_MISO, GPIO_IN);
    gpio_disable_pulls(PIN_SNIFF_CS);
    gpio_disable_pulls(PIN_SNIFF_SCK);
    gpio_disable_pulls(PIN_SNIFF_MOSI);
    gpio_disable_pulls(PIN_SNIFF_MISO);

    /* ---- 3) 等总线空闲（CS 为高）----
     * 最多等 1 秒：如果 CS 一直低（线接错、被测设备没上电），
     * 也不要卡死在这里，继续初始化即可，后面主循环会处理异常。 */
    uint64_t wait_start = time_us_64();
    while (!gpio_get(PIN_SNIFF_CS) && (time_us_64() - wait_start) < 1000000u) {
        tight_loop_contents();
    }

    PIO pio = pio0;

    /* RP2350 的 PIO 有自己的 GPIO 窗口基址（RP2350A 上为 0 即可）。
     * 归零后 PIO 内部的引脚号就是真实 GPIO 号：IN_BASE=17 直接对应
     * GP17~GP20。 */
    pio->gpiobase = 0;

    uint sm = pio_claim_unused_sm(pio, true);
    uint offset = pio_add_program(pio, &spi_oversample_program);

    /* ---- 5) 自检：程序必须恰好是那一条 `in pins,4` ----
     * 一旦 .pio 被改坏（少采一根线、采样根数变了），屏幕上先亮 ERROR，
     * 省得对着"数据不对"瞎猜。用 SDK 自己的编码器生成期望值，
     * 不依赖手算的指令码（0x4004）。 */
    if (spi_oversample_program.length != 1 ||
        spi_oversample_program.instructions[0] != pio_encode_in(pio_pins, 4)) {
        g_sniff.state = SNIFF_ERROR;
        __dmb();
        while (true) {
            tight_loop_contents();
        }
    }

    /* 把这四个引脚的功能切换到 PIO。输入路径本身不依赖 GPIO 复用功能，
     * 但这一步会确保 pad 的输入使能、隔离解除都处于正确状态。 */
    pio_gpio_init(pio, PIN_SNIFF_CS);
    pio_gpio_init(pio, PIN_SNIFF_SCK);
    pio_gpio_init(pio, PIN_SNIFF_MOSI);
    pio_gpio_init(pio, PIN_SNIFF_MISO);

    /* ---- 4) 状态机配置 ----
     *   in pins,4 的起始脚 = CS（IN_BASE），四脚连续；
     *   右移 + 满 32 bit 自动 push -> 每 8 个采样点产生一个 32bit 字；
     *   RX FIFO 并成 8 级 -> 25M 字/s 下可缓冲 320ns，DMA 偶尔被总线
     *   挤占时不至于立刻溢出丢采样；
     *   clkdiv = 1（默认值）-> 每个 clk_sys 周期采一个点，即 5ns/点。 */
    pio_sm_config cfg = spi_oversample_program_get_default_config(offset);
    sm_config_set_in_pins(&cfg, PIN_SNIFF_CS);   /* IN_BASE -> CS，四脚连续 */
    sm_config_set_in_shift(&cfg, true, true, 32);/* 右移，满 32 bit 自动 push */
    sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &cfg);

    /* 用 SET PINDIRS 指令把四脚设为输入 */
    pio_sm_set_consecutive_pindirs(pio, sm, PIN_SNIFF_CS, 4, false);

#if PICO_RP2350
    /* RP2350 的 PIO 输入同步器有 2 拍延迟，会削掉 40MHz 信号的窄脉冲。
     * 过采样阶段直接旁路，拿到的是焊盘上的原始电平。
     * 两个参数分别是"置 1 的旁路位"和"要更新的位掩码"。 */
    uint32_t bypass = (1u << PIN_SNIFF_CS) | (1u << PIN_SNIFF_SCK) |
                      (1u << PIN_SNIFF_MOSI) | (1u << PIN_SNIFF_MISO);
    pio_set_input_sync_bypass_with_mask(pio, bypass, bypass);
#endif

    /* ---- 6) 实测 clk_sys ----
     * 必须在启动采样 DMA 之前做：测量期间会占用一个状态机 0.25s，
     * 先测完再开采样，采样流里就不会混进测量造成的"断档"。 */
    g_sniff.clk_meas_khz = measure_clk_sys_khz(pio);

    /* ---- 7) DMA：PIO RX FIFO -> 环形缓冲 ----
     *   32 位传输（一个字正好是 8 个采样点）；
     *   读地址不递增（始终读同一个 FIFO 寄存器）；
     *   写地址递增；
     *   DREQ 来自 PIO 的 RX（FIFO 有数据才搬）；
     *   ring size 15 -> 写地址的低 15 位（32KB）自动回绕；
     *   ENDLESS 传输计数 -> 通道永不停止。
     * RP2350 的传输计数高 4 位是模式位，ENDLESS 的编码是 0xFFFFFFFF，
     * 但必须用 SDK 的编码函数生成，不能自己随手写一个"很大的数"。 */
    s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, false);
    channel_config_set_write_increment(&dma_cfg, true);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(pio, sm, false));
    channel_config_set_ring(&dma_cfg, true, 15);
    /* 采样 DMA 优先于 Core 0 的普通总线访问，减少被挤占导致的 FIFO 溢出 */
    channel_config_set_high_priority(&dma_cfg, true);
    dma_channel_configure(s_dma_chan, &dma_cfg,
                          s_ring, &pio->rxf[sm],
                          dma_encode_endless_transfer_count(), true);

    /* ---- 8) 启动采样，进入主循环 ---- */
    pio_sm_set_enabled(pio, sm, true);

    uint32_t frame = 0;
    g_sniff.state = SNIFF_ARMED;
    __dmb();

    /* ---- 9) CS 下降沿中断 ----
     * 必须放在 Core 1 上注册：中断处理函数由当前核执行，Core 1 才是
     * 负责采集的核。
     *
     * 为什么不用"轮询 CS 引脚"：解析一帧（找边沿、逐点统计、必要时
     * 拷 4KB 原始数据）要几十到几百微秒，这段时间里轮询循环是停住的。
     * 而 ESP32 读完状态寄存器（16 bit，约 400ns）后紧接着就发 64 字节
     * 读命令（约 14µs），如果解析上一帧时正好错过这段，这个长帧就
     * 永远看不到了。中断不受主循环在干什么影响，先把"有过下降沿"
     * 记下来，主循环再慢慢去环形缓冲里取——缓冲有 328µs，来得及。 */
    gpio_set_irq_enabled_with_callback(PIN_SNIFF_CS, GPIO_IRQ_EDGE_FALL,
                                       true, &cs_fall_isr);

    /* 上一帧的起点位置，用来识别"同一个帧被振铃重复唤醒"的情况：
     * 帧中间 CS 的短毛刺会再产生下降沿，主循环处理完这一帧后可能
     * 又看到事件标志，若重新解析会得到同一帧，导致重复发布。 */
    static uint32_t s_last_start;
    static bool s_have_last_start;

    for (;;) {
        /* 等一帧开始。事件标志由中断置起；就算 CPU 正在忙于解析
         * 上一帧，新的下降沿也已经记下了，不会漏帧。 */
        if (!s_cs_event) {
            tight_loop_contents();
            continue;
        }
        /* 关中断取"事件时间"并清标志：s_cs_fall_us 是 64 位，在 32 位核上
         * 读取不是原子的，若恰好被新的下降沿中断打断会读到半新半旧的值。
         * 关中断只有几条指令，期间发生的下降沿会在开中断后立刻补上标志。 */
        uint32_t irq_save = save_and_disable_interrupts();
        uint64_t t_fall = s_cs_fall_us;
        s_cs_event = false;
        restore_interrupts(irq_save);

        /* 等 CS 连续为高 FRAME_END_US（5µs）才算这一帧结束。
         * 只看到一次高电平不算：边沿振铃会让 CS 瞬间回高又落下。
         *
         * 如果进到这里时 CS 已经是高，说明这一帧在解析上一帧期间
         * 就已经开始并结束了——直接去解析，不用再等。
         *
         * CAPTURE 状态只在"CS 仍为低、确实在等帧尾"时显示。若进到这里
         * CS 已经回高，说明该事件早已处理完，不再闪一下 CAPTURE——之前
         * 串口里大量"CAPTURE 了但没数据"就是这些迟到事件造成的。 */
        bool saw_rise = false;    /* 我们亲眼看到 CS 回高（计时才准） */
        bool ended = false;       /* 当前 CS 已经回高（帧完整结束） */
        bool in_high = false;
        uint64_t high_since = 0;

        if (gpio_get(PIN_SNIFF_CS)) {
            ended = true;
            high_since = time_us_64();
        } else {
            g_sniff.state = SNIFF_CAPTURING;
            __dmb();
            for (;;) {
                uint64_t now = time_us_64();
                if (gpio_get(PIN_SNIFF_CS)) {
                    if (!in_high) {
                        in_high = true;
                        high_since = now;
                    } else if ((now - high_since) >= FRAME_END_US) {
                        saw_rise = true;
                        ended = true;
                        break;
                    }
                } else {
                    in_high = false;
                }
                if ((now - t_fall) > FRAME_TIMEOUT_US) {
                    break;            /* 兜底：CS 一直不回高 */
                }
            }
        }

        /* 回头在环形缓冲里解析这一帧。注意即使 decoded=false，
         * 下面的诊断字段也会被更新，方便 Core 0 从串口定位问题。 */
        frame_result_t r;
        bool decoded = decode_last_frame(&r);

        /* 帧中间 CS 的毛刺也会触发下降沿中断。处理完一帧后如果又看到
         * 事件，重新解析得到的还是同一帧（起点位置相同），直接跳过，
         * 避免重复计数、重复发布。 */
        if (decoded && s_have_last_start && r.start == s_last_start) {
            __dmb();
            g_sniff.state = s_published_state;
            continue;
        }
        if (decoded) {
            s_last_start = r.start;
            s_have_last_start = true;
        }

        /* PIO RX FIFO 溢出标志：说明 DMA 有一段时间没跟上采样速度。
         * 读之后要写 1 清零（fdebug 是写 1 清零的）。 */
        bool stall = ((pio->fdebug >> (PIO_FDEBUG_RXSTALL_LSB + sm)) & 1u) != 0;
        pio->fdebug = 1u << (PIO_FDEBUG_RXSTALL_LSB + sm);

        /* 帧时长：优先用"亲眼看到 CS 回高"的墙钟时间；如果是忙完才
         * 来解析（没看到回高），就按帧内采样点数换算——200 个点 = 1µs，
         * 这个值反而比墙钟更准。 */
        uint32_t elapsed_us = 0;
        if (decoded) {
            if (saw_rise && high_since > t_fall) {
                elapsed_us = (uint32_t)(high_since - t_fall);
            } else {
                elapsed_us = (r.samples + 100u) / 200u;
            }
        }
        /* full：撞到解析上限；hit_head：帧尾还没写进环形缓冲。
         * 两种都属于"帧不完整"，发布时统一标 frame_full=1，不要当成
         * 一帧好数据。 */
        bool full = decoded && ((r.samples >= MAX_FRAME_SAMPLES) || r.hit_head);

        g_sniff.rxstall = stall ? 1u : 0u;

        /* 最近一帧的原始结果单独存，供诊断显示；它不参与屏幕上的
         * "已发布结果"，所以短帧/毛刺冲不掉上一次的好数据。 */
        g_sniff.last_bits = decoded ? r.bits : 0u;
        g_sniff.last_samples = decoded ? r.samples : 0u;
        g_sniff.last_decoded = decoded ? 1u : 0u;

        /* 诊断：把环形缓冲里最新的一段原始字拷到共享区，由 Core 0 打到
         * 串口。触发条件分两类、各自限量，避免刷屏，也避免开机时的两次
         * 失败把长帧的转储名额挤掉：
         *   - 解码失败：最多 2 次，看看环里到底是什么波形；
         *   - 长帧（≥ SNIFF_MIN_BITS）：最多 1 次，把真正关心的 64 字节
         *     读帧原样留下，用来逐点核对 SCK/MOSI/MISO 的相位关系。
         * 双缓冲写 raw[slot]：Core 0 按 raw_slot 读完整的一份，不会被
         * 下一段转储从中间改掉。 */
        bool dump_fail = !decoded && (s_raw_fail_dumps < 2u);
        bool dump_long = decoded && (r.bits >= SNIFF_MIN_BITS) &&
                         (s_raw_long_dumps < 1u);
        if (dump_fail || dump_long) {
            if (dump_fail) {
                s_raw_fail_dumps++;
            } else {
                s_raw_long_dumps++;
            }
            uint32_t slot = s_raw_seq & 1u;
            uint32_t wi_from;
            if (dump_long) {
                /* 长帧转储按帧起点对齐，往前多留 16 个字（640ns）的空闲，
                 * 这样能看到 CS 下降沿本身。不能像失败转储那样从写指针
                 * 往前 1024 字截：主循环处理其他事件耽误一会儿，写指针就
                 * 会跑出 41µs 的窗口，把真正要看的这一帧挤掉。 */
                wi_from = ((r.start >> 3) + RING_WORDS - 16u) & (RING_WORDS - 1u);
            } else {
                uint32_t wi_now = (g_sniff.ring_head >> 3) & (RING_WORDS - 1u);
                wi_from = (wi_now + RING_WORDS - SNIFF_RAW_WORDS) & (RING_WORDS - 1u);
            }
            uint32_t nz = 0;
            for (uint32_t i = 0; i < SNIFF_RAW_WORDS; i++) {
                uint32_t w = s_ring[(wi_from + i) & (RING_WORDS - 1u)];
                g_sniff.raw[slot][i] = w;
                if (w) {
                    nz++;
                }
            }
            g_sniff.raw_nonzero = nz;
            g_sniff.raw_count = SNIFF_RAW_WORDS;
            g_sniff.raw_kind = dump_long ? 1u : 0u;
            g_sniff.raw_bits = decoded ? r.bits : 0u;
            g_sniff.raw_samples = decoded ? r.samples : 0u;
            g_sniff.raw_seen = g_sniff.frames_seen + 1u;
            g_sniff.raw_slot = slot;
            s_raw_seq++;
            __dmb();
            g_sniff.raw_seq = s_raw_seq;
        }

        /* ---- 诊断快照：把这一帧的所有字段一次性交给 Core 0 ----
         * 双缓冲 + 序号：Core 0 只在 diag_seq 变化时读，且读的是已经
         * 写完的那一份，不会出现"前几个字段是这一帧、后几个字段已经
         * 被下一帧覆盖"的串帧现象（之前 evt 行里的数值自相矛盾就是
         * 这个原因）。 */
        {
            uint32_t di = s_diag_seq & 1u;
            g_sniff.diag[di].seen    = g_sniff.frames_seen + 1u;
            g_sniff.diag[di].decoded = decoded ? 1u : 0u;
            g_sniff.diag[di].bits    = decoded ? r.bits : 0u;
            g_sniff.diag[di].samples = decoded ? r.samples : 0u;
            g_sniff.diag[di].csh     = g_sniff.ring_cs_hi;
            g_sniff.diag[di].tail    = g_sniff.ring_tail;
            g_sniff.diag[di].head    = g_sniff.ring_head;
            g_sniff.diag[di].waddr   = g_sniff.dma_waddr;
            g_sniff.diag[di].stall   = stall ? 1u : 0u;
            g_sniff.diag[di].elapsed = elapsed_us;
            g_sniff.diag[di].full    = full ? 1u : 0u;
            g_sniff.diag[di].end_run = decoded ? r.end_run : 0u;
            g_sniff.diag[di].hit_head = decoded ? r.hit_head : 0u;
            g_sniff.diag[di].saw     = saw_rise ? 1u : 0u;
            g_sniff.diag[di].cand    = r.cand;
            g_sniff.diag[di].max_low = r.max_low;
            g_sniff.diag[di].max_high = r.max_high;
            __dmb();
            s_diag_seq++;
            g_sniff.diag_seq = s_diag_seq;
        }

        /* frames_seen 最后加：Core 0 看到它变化时，上面的诊断字段已经就绪 */
        __dmb();
        g_sniff.frames_seen = g_sniff.frames_seen + 1u;

        /* 短帧（写使能 0x06、擦除 0x20 等命令帧）只更新诊断字段，
         * 不刷新已发布结果——否则屏幕会被 8 bit 的命令帧反复覆盖。 */
#if SNIFF_MIN_BITS > 0
        bool short_frame = !decoded || (r.bits < SNIFF_MIN_BITS);
#else
        bool short_frame = !decoded;
#endif

        if (decoded && !short_frame) {
            publish_frame(&r, elapsed_us, ended, full, ++frame);
            s_published_state = ended ? SNIFF_DONE : SNIFF_TIMEOUT;
            /* 屏障必须在写 state 之前：保证 Core 0 看到新 state 时，
             * 结果字段已经全部落进 SRAM。 */
            __dmb();
            g_sniff.state = s_published_state;
            multicore_fifo_push_blocking(0xD0DEu);   /* 通知 Core 0 */
        } else {
            /* 被过滤的命令帧：把状态行拉回最后一次发布的结果，
             * 否则界面会一直停在 CAPTURING 上。 */
            __dmb();
            g_sniff.state = s_published_state;
        }
    }
}
