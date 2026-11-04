/*
 * 手持式多协议综合分析仪（第一项验证：40MHz SPI 从机监听）
 * Core 0 —— 系统时钟、ST7789 屏幕、串口日志
 *
 * 双核分工：
 *   Core 1（drivers/sniffer）用 PIO 以 200MHz 过采样四根 SPI 线，DMA 把
 *   采样流连续写进环形缓冲；CS 每出现一帧，就回头在缓冲里把这一帧解出来，
 *   结果放进全局 g_sniff。
 *   Core 0 只负责"使用"这些结果：每 20ms 刷新一次屏幕；帧号变化时把统计值
 *   和完整字节流打到串口，供逐字节核对。
 *
 * 跨核数据约定（两个核必须遵守同一套顺序，否则会读到半新半旧的数据）：
 *   写方（Core 1）：先填完所有结果字段 → __dmb() → 最后写 state / frame_no
 *                   这类"标志字段"；
 *   读方（Core 0）：发现标志变化 → __dmb() → 再读其余字段。
 *   __dmb() 是数据存储屏障，保证屏障之前的写操作对另一个核可见之后，
 *   屏障之后的写才会发生。这里用它实现"发布/订阅"式的无锁共享。
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "board.h"
#include "sniffer.h"
#include "st7789.h"

/* 屏幕一行最多容纳 22 个字符（135 像素宽 / 6 像素字宽），超出的部分截掉 */
#define UI_CHARS 22
/* 字体缩放倍数：1 = 6x8 点阵原尺寸 */
#define UI_SCALE 1

/* 把内部状态枚举翻译成屏幕/串口上显示的英文单词 */
static const char *state_name(uint32_t state)
{
    switch (state) {
    case SNIFF_IDLE:      return "IDLE";     /* 刚上电，还没开始 */
    case SNIFF_ARMED:     return "ARMED";    /* 采样已就绪，等 CS 拉低 */
    case SNIFF_CAPTURING: return "CAPTURE";  /* CS 处于低电平，正在等帧结束 */
    case SNIFF_DONE:      return "DONE";     /* 最近一帧正常收完并已发布 */
    case SNIFF_TIMEOUT:   return "TIMEOUT";  /* 收完但 CS 一直没回高，按超时收尾 */
    case SNIFF_ERROR:     return "ERROR";    /* 初始化自检失败，不再工作 */
    default:              return "?";
    }
}

/*
 * 在屏幕的固定行上打印一行文本。
 *
 * 参数：
 *   y    —— 行首的像素纵坐标；
 *   fg   —— 前景色（16 位 RGB565，见 st7789.h 的 LCD_xxx 宏）；
 *   fmt  —— printf 风格的格式串，后面的可变参数与之对应。
 *
 * 为什么不用 st7789_draw_text 直接画：
 *   ST7789 画字只是往像素里写前景色，不会清背景。若新字符串比旧的短，
 *   旧字符的尾巴会留在屏幕上（比如 "DONE" 换成 "IDLE" 会变成 "IDLE" 后面
 *   还挂着 "E"）。所以这里统一按 UI_CHARS 补空格，把整行 22 个字符都重画，
 *   旧内容自然被覆盖掉。
 */
static void draw_line(int y, uint16_t fg, const char *fmt, ...)
{
    char text[64];                 /* 格式化后的原始文本 */
    char line[UI_CHARS + 1];       /* 补空格后的定宽行，末尾留 '\0' */

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    size_t n = strlen(text);
    if (n > UI_CHARS) {
        n = UI_CHARS;              /* 太长就截断，保证不会画到屏幕外 */
    }
    memcpy(line, text, n);
    for (size_t i = n; i < UI_CHARS; i++) {
        line[i] = ' ';
    }
    line[UI_CHARS] = '\0';

    st7789_draw_text(2, y, line, fg, LCD_BLACK, UI_SCALE);
}

/*
 * 把前 count 个字节格式化成 "AA BB CC ..." 字符串，供屏幕显示。
 *
 * 参数：
 *   out     —— 输出缓冲区，必须以 '\0' 结尾；
 *   out_len —— 输出缓冲区长度；
 *   bytes   —— 字节来源。类型是 volatile uint32_t*，因为共享区里的字段
 *              由 Core 1 异步更新，不能按普通变量优化；
 *   count   —— 最多格式化的字节数。
 *
 * 屏幕一行只有 22 个字符，每个字节占 "XX " 三个字符，所以实际上 7 个字节
 * 刚好放得下；调用方传 7 即可。pos + 4 < out_len 预留了 "XX " 加结尾的空间。
 *
 * 末尾会去掉最后一个空格（循环里每个字节后面都跟了一个空格）。
 */
static void format_bytes(char *out, size_t out_len,
                         const volatile uint32_t *bytes, size_t count)
{
    size_t pos = 0;
    out[0] = '\0';
    for (size_t i = 0; i < count && pos + 4 < out_len; i++) {
        pos += (size_t)snprintf(out + pos, out_len - pos,
                                "%02X ", (unsigned)bytes[i]);
    }
    if (pos > 0) {
        out[pos - 1] = '\0';
    }
}

/*
 * 串口打印用的"本帧字节流快照"。
 *
 * 为什么不直接从 g_sniff.cap_mosi 打印：
 *   串口输出（尤其 UART 115200）比抓包慢几个数量级。如果一边打印一边读
 *   共享区，Core 1 可能在中途发布下一帧，把 cap_mosi 改掉，结果就是前半截
 *   是旧帧、后半截是新帧。所以先用一个循环把整帧拷到这两个本地缓冲，
 *   之后 Core 1 怎么覆盖共享区都与打印无关。
 */
static uint8_t s_tx_mosi[SNIFF_DUMP_BYTES];
static uint8_t s_tx_miso[SNIFF_DUMP_BYTES];

/*
 * 原始采样转储的本地快照。
 * Core 1 用 raw[2][...] 双缓冲更新，这里先把整段 4KB 拷出来再打印，
 * 保证打印的内容来自同一次转储；直接边打印边读共享区会被下一段转储
 * 从中间覆盖（之前 raw 内容和 evt 对不上就是这个原因）。
 */
static uint32_t s_raw_snap[SNIFF_RAW_WORDS];

/*
 * 以"十六进制 + ASCII"的形式打印一段字节流，每行 16 字节。
 *
 * 参数：
 *   tag —— 行首标签，传 "MOSI" 或 "MISO"；
 *   buf —— 要打印的数据；
 *   n   —— 字节数。
 *
 * 每行格式：
 *   MOSI 0010: 00 01 02 ...                                  |................|
 *   ^标签 ^偏移  ^十六进制（不足 16 字节补空格对齐）          ^可打印字符，其余用 '.'
 *
 * ASCII 一栏方便人眼直接看出 ESP32 写入的 "Hello W25Q64! Counter = N" 这类文本。
 */
static void dump_bytes(const char *tag, const uint8_t *buf, uint32_t n)
{
    for (uint32_t off = 0; off < n; off += 16u) {
        uint32_t cnt = (n - off < 16u) ? (n - off) : 16u;

        printf("%s %04lX:", tag, (unsigned long)off);
        for (uint32_t i = 0; i < cnt; i++) {
            printf(" %02X", (unsigned)buf[off + i]);
        }
        for (uint32_t i = cnt; i < 16u; i++) {
            printf("   ");                 /* 末行不足 16 字节时对齐 ASCII 栏 */
        }
        printf("  |");
        for (uint32_t i = 0; i < cnt; i++) {
            uint8_t c = buf[off + i];
            putchar((c >= 0x20u && c < 0x7Fu) ? c : '.');   /* 不可打印字符画 '.' */
        }
        printf("|\n");
    }
}

/* 状态对应的显示颜色：完成绿、出错红、采集中黄、其余青色 */
static uint16_t state_color(uint32_t state)
{
    switch (state) {
    case SNIFF_DONE:      return LCD_GREEN;
    case SNIFF_TIMEOUT:
    case SNIFF_ERROR:     return LCD_RED;
    case SNIFF_CAPTURING: return LCD_YELLOW;
    default:              return LCD_CYAN;
    }
}

/*
 * 刷新整屏。被 main() 每 20ms 调用一次（约 50Hz）。
 *
 * 关键点：一进入函数就先 __dmb()，再把要用的字段逐个读进局部变量。
 * 这样整屏显示的是同一时刻的一份"快照"；如果每条 draw_line 都直接读
 * g_sniff，Core 1 中途发布新帧就会导致上半屏是旧帧、下半屏是新帧。
 */
static void render(void)
{
    __dmb();                          /* 保证先看到 Core 1 已经写好的数据 */
    uint32_t state = g_sniff.state;
    uint32_t bits = g_sniff.frame_bits;
    uint32_t bytes_line = g_sniff.bytes_line;
    uint32_t frame = g_sniff.frame_no;
    uint32_t seen = g_sniff.frames_seen;
    uint32_t elapsed = g_sniff.elapsed_us;
    uint32_t stall = g_sniff.rxstall;
    uint32_t last_bits = g_sniff.last_bits;
    uint32_t last_samples = g_sniff.last_samples;
    uint32_t hi_min = g_sniff.sck_hi_min;
    uint32_t hi_max = g_sniff.sck_hi_max;

    /* CLK：Core 1 启动时用 XOSC 时基实测的 clk_sys。
     * 0 表示还没测完（约 0.25s），此时不显示，避免闪一个 0。 */
    if (g_sniff.clk_meas_khz) {
        draw_line(22, LCD_GREY, "CLK %lu.%03lu MHZ",
                  (unsigned long)(g_sniff.clk_meas_khz / 1000u),
                  (unsigned long)(g_sniff.clk_meas_khz % 1000u));
    }

    /* STATE：Core 1 的采集状态机（见 state_name 的说明） */
    draw_line(34, state_color(state), "STATE %s", state_name(state));

    /* CS/SCK：直接读引脚当前电平。用来确认接线和总线空闲状态：
     * 没有帧时 CS 应为 HIGH；SPI Mode 0 下空闲 SCK 为 LOW。 */
    draw_line(46, LCD_WHITE, "CS %s  SCK %s",
              gpio_get(PIN_SNIFF_CS) ? "HIGH" : "LOW",
              gpio_get(PIN_SNIFF_SCK) ? "HIGH" : "LOW");

    /* FRAME 已发布帧数 / 检测到的 CS 帧总数。
     * 两个数差得远，说明大量短帧（写使能、擦除等命令帧）被过滤了，属正常。 */
    draw_line(58, LCD_WHITE, "FRAME %lu/%lu",
              (unsigned long)frame, (unsigned long)seen);

    /* BITS：本帧 SCK 上升沿个数（= 每条线的 bit 数）；后面是完整字节数。
     * 一次 0x0B 读 64 字节应为 552 bit / 69 字节。 */
    draw_line(70, LCD_WHITE, "BITS %lu %luB",
              (unsigned long)bits, (unsigned long)bytes_line);

    /* RESULT：对已发布帧的结论，只有在 DONE 状态下才有意义 */
    const char *result = "WAIT";
    uint16_t result_color = LCD_GREY;
    if (state == SNIFF_DONE) {
        if (stall) {
            result = "FAIL STALL";    /* PIO RX FIFO 溢出过：DMA 没跟上采样 */
            result_color = LCD_RED;
        } else if (bits == 0) {
            result = "NO DATA";       /* 有 CS 帧，但一个 SCK 边沿都没采到 */
            result_color = LCD_RED;
        } else {
            result = "PASS";
            result_color = LCD_GREEN;
        }
    } else if (state == SNIFF_TIMEOUT || state == SNIFF_ERROR) {
        result = "FAIL";
        result_color = LCD_RED;
    }
    draw_line(82, result_color, "RESULT %s", result);

    /* TIME：CS 下降沿到 CS 连续回高之间的时间，单位 µs（XOSC 时基） */
    if (elapsed) {
        draw_line(94, LCD_WHITE, "TIME %lu US", (unsigned long)elapsed);
    } else {
        draw_line(94, LCD_GREY, "TIME ----");
    }

    /* SCKHI：SCK 高电平宽度的最小值/最大值，单位是 5ns 采样点。
     * 40MHz、50% 占空比时每个高电平约 12.5ns，即 2~3 个采样点。
     * 最小值掉到 1（红色）说明高电平已经窄到一个采样点，边沿开始退化；
     * 最大值很大则说明存在帧间空闲的高电平段。 */
    if (hi_max) {
        draw_line(106, (hi_min <= 1u) ? LCD_RED : LCD_WHITE,
                  "SCKHI %lu/%lu", (unsigned long)hi_min,
                  (unsigned long)hi_max);
    } else {
        draw_line(106, LCD_GREY, "SCKHI ----");
    }

    /* CSH：最近 4096 个采样点（20µs）里 CS 为高的个数。
     * 空闲时应该接近 4096；为 0 说明环形缓冲没收到数据（DMA 没在搬）
     * 或者 PIO 采到的 CS 恒为低。 */
    draw_line(118, LCD_GREY, "CSH %lu/4096", (unsigned long)g_sniff.ring_cs_hi);

    /* MOSI/MISO：已发布帧的前 7 个字节（屏幕一行放得下 7 组 "XX "） */
    char bytes[32];

    draw_line(130, LCD_CYAN, "MOSI");
    format_bytes(bytes, sizeof(bytes), g_sniff.first_mosi, 7);
    draw_line(142, LCD_WHITE, "%s", bytes);

    draw_line(154, LCD_CYAN, "MISO");
    format_bytes(bytes, sizeof(bytes), g_sniff.first_miso, 7);
    draw_line(166, LCD_WHITE, "%s", bytes);

    /* HASH：整帧 MOSI/MISO 的 FNV-1a 哈希。
     * 同一地址反复读时，MISO 的哈希每帧都应该相同；不同则说明采集不稳定。 */
    draw_line(178, LCD_CYAN, "HASH MOSI");
    draw_line(190, LCD_WHITE, "%08lX", (unsigned long)g_sniff.hash_mosi);

    draw_line(202, LCD_CYAN, "HASH MISO");
    draw_line(214, LCD_WHITE, "%08lX", (unsigned long)g_sniff.hash_miso);

    /* LAST：最近一次解码尝试的结果（包括被过滤的短帧，用于诊断）：
     *   "LAST 552b 2763s" —— 解出了 552 bit、帧内 2763 个采样点；
     *   "LAST -- XXXXXXXX" —— 最近 82µs 内没找到稳定的 CS 下降沿，
     *                        后面的 16 进制是最近 8 个采样点的原始 4bit 字段
     *                        （每个 nibble 依次是 [MISO MOSI SCK CS]）。
     * 空闲时 8 个 nibble 应该是 1 或 9（只可能 CS=1、其余为 0）。 */
    if (g_sniff.last_decoded) {
        draw_line(226, LCD_GREY, "LAST %lub %lus",
                  (unsigned long)last_bits, (unsigned long)last_samples);
    } else {
        draw_line(226, LCD_RED, "LAST -- %08lX",
                  (unsigned long)g_sniff.ring_tail);
    }
}

int main(void)
{
    /* set_sys_clock_khz 里第二个参数 true 表示"同时把外设时钟一起重新分配"。
     * 200MHz 是过采样能覆盖 40MHz SPI（25ns 位周期、5ns/采样点）的前提。 */
    bool clk_ok = set_sys_clock_khz(SNIFF_SYS_CLK_KHZ, true);

    /* 打开 stdio：本工程同时开了 UART0(GP0/GP1, 115200) 和 USB CDC，
     * printf 会两边都输出（配置见 CMakeLists.txt）。 */
    stdio_init_all();

    st7789_init();
    st7789_fill(LCD_BLACK);

    /* 构建时间戳：开机串口第一行就能确认板子上跑的到底是不是最新固件。
     * 这个字符串由 CMake 每次构建时生成（cmake/gen_build_stamp.cmake），
     * 不是 __DATE__/__TIME__——后者只在包含它的本文件被重新编译时才变，
     * 只改 sniffer.c 时会一直显示旧时间，容易误判成“烧录没成功”。 */
    extern const char fw_build_stamp[];
    printf("build %s\n", fw_build_stamp);

    st7789_draw_text(14, 2, "SPI SNIFF", LCD_WHITE, LCD_BLACK, 2);

    /* 标题下方显示实际配置的频率（读时钟树寄存器，是"设定值"；
     * 屏幕第 22 行的 CLK 才是 Core 1 用 XOSC 实测的真实值）。 */
    uint32_t sys_mhz = clock_get_hz(clk_sys) / 1000000u;
    char sub[24];
    snprintf(sub, sizeof(sub), "CORE1 OVERSAMPLE %luMHZ", (unsigned long)sys_mhz);
    st7789_draw_text(2, 22, clk_ok ? sub : "CLK FAIL",
                     clk_ok ? LCD_GREY : LCD_RED, LCD_BLACK, 1);

    printf("RP_Bus_Sniffer: sys=%lu MHz clk_ok=%d\n",
           (unsigned long)sys_mhz, clk_ok);
    printf("sniffer pins: CS=%d SCK=%d MOSI=%d MISO=%d\n",
           PIN_SNIFF_CS, PIN_SNIFF_SCK, PIN_SNIFF_MOSI, PIN_SNIFF_MISO);

    /* 启动 Core 1。入口是 sniffer_core1_entry()，它会一直循环不返回。 */
    multicore_launch_core1(sniffer_core1_entry);

    uint32_t last_state = 0xFFFFFFFFu;
    uint32_t last_frame = 0;
    uint32_t last_diag_seq = 0;
    uint32_t last_raw_seq = 0;
    uint64_t last_evt_us = 0;

    while (true) {
        render();

        /* 硬件 mailbox（FIFO）上的通知消息。约定 Core 1 发布一帧就 push
         * 0xD0DE；真正的数据在 g_sniff 共享区，不经过 FIFO。
         * 这里只把消息打印出来，用于确认 Core 1 还活着。 */
        while (multicore_fifo_rvalid()) {
            uint32_t msg = multicore_fifo_pop_blocking();
            printf("core1 msg 0x%08lX\n", (unsigned long)msg);
        }

        /* 状态变化时打一行日志（比屏幕更容易翻历史） */
        __dmb();
        uint32_t state = g_sniff.state;
        if (state != last_state) {
            printf("state -> %s\n", state_name(state));
            last_state = state;
        }

        /* 每检测到一个 CS 帧（包括被过滤的短帧）打一行诊断。
         *
         * 数据来自 Core 1 的双缓冲快照：diag_seq 变化表示有一份完整的新
         * 快照，读 diag[seq & 1] 得到的所有字段一定属于同一个 CS 帧。
         * （早先逐字段读共享区，打印过程中 Core 1 又处理了下一帧，日志里
         *   会出现自相矛盾的数值，反而误导排查。）
         *
         * 字段含义：
         *   dec    —— 是否成功解出帧（0 = 环里没找到稳定 CS 下降沿）
         *   lbits  —— 该帧解出的 bit 数
         *   lsmpl  —— 该帧在采样流里的长度（采样点个数）
         *   csh    —— 最近 4096 个采样点里 CS 为高的个数
         *   tail   —— 最近 8 个采样点的原始 4bit 字段
         *   head   —— DMA 写指针（采样点位置），两次事件之间必须一直在变
         *   waddr  —— DMA 写地址的原始值，便于发现地址没落在环内
         *   us     —— CS 下降到帧结束的时间
         *   stall  —— PIO RX FIFO 是否溢出过
         *   full   —— 帧长是否超过解析上限
         * 判断方法：
         *   head 不变          -> DMA 没在搬数据（PIO 没输出或 DMA 没启动）
         *   csh = 0 或 4096    -> PIO 采到的 CS 恒低/恒高，引脚映射可疑
         *   dec=0 但 head 在变 -> 采样在跑，问题在解码条件
         *
         * 刷屏控制：擦除期间 ESP32 会连续轮询状态寄存器（每帧约 400ns），
         * 每帧都打会把 115200 的串口堵死。所以长帧（≥ SNIFF_MIN_BITS）
         * 必打；短帧和失败帧最多每 500ms 打一行。 */
        __dmb();
        uint32_t dseq = g_sniff.diag_seq;
        if (dseq != last_diag_seq) {
            last_diag_seq = dseq;
            const volatile sniff_diag_t *d = &g_sniff.diag[dseq & 1u];
            uint32_t now_us = (uint32_t)time_us_64();

            bool long_frame = d->decoded && (d->bits >= SNIFF_MIN_BITS);
            if (long_frame || (uint32_t)(now_us - (uint32_t)last_evt_us) >= 500000u) {
                last_evt_us = now_us;
                printf("evt %lu dec=%lu lbits=%lu lsmpl=%lu csh=%lu "
                       "tail=%08lX head=%lu waddr=%08lX us=%lu stall=%lu "
                       "full=%lu end=%lu hh=%lu saw=%lu cand=%lu "
                       "ml=%lu mh=%lu\n",
                       (unsigned long)d->seen,
                       (unsigned long)d->decoded,
                       (unsigned long)d->bits,
                       (unsigned long)d->samples,
                       (unsigned long)d->csh,
                       (unsigned long)d->tail,
                       (unsigned long)d->head,
                       (unsigned long)d->waddr,
                       (unsigned long)d->elapsed,
                       (unsigned long)d->stall,
                       (unsigned long)d->full,
                       (unsigned long)d->end_run,
                       (unsigned long)d->hit_head,
                       (unsigned long)d->saw,
                       (unsigned long)d->cand,
                       (unsigned long)d->max_low,
                       (unsigned long)d->max_high);
            }
        }

        /* 解码失败时 Core 1 会拷一段原始采样字过来（前两次）。
         * 每行 8 个字，一个字 = 8 个采样点，按时间从旧到新排列。
         * 字的 nibble 布局：bit[4k+0]=CS, +1=SCK, +2=MOSI, +3=MISO；
         * k=0 是该字最早的采样点。
         * nonzero 是这段里非零字的个数：0 说明环形缓冲里根本没有数据。 */
        __dmb();
        uint32_t raw_seq = g_sniff.raw_seq;
        if (raw_seq != last_raw_seq) {
            last_raw_seq = raw_seq;

            /* 元数据和数据都先抓到本地再打印：Core 1 下一次转储会覆盖
             * 共享区，打印本身又很慢（4KB 在 115200 下约 1 秒）。 */
            uint32_t slot   = g_sniff.raw_slot & 1u;
            uint32_t kind   = g_sniff.raw_kind;
            uint32_t rbits  = g_sniff.raw_bits;
            uint32_t rsmpl  = g_sniff.raw_samples;
            uint32_t rseen  = g_sniff.raw_seen;
            uint32_t rnz    = g_sniff.raw_nonzero;
            uint32_t n      = g_sniff.raw_count;
            if (n > SNIFF_RAW_WORDS) {
                n = SNIFF_RAW_WORDS;
            }
            for (uint32_t i = 0; i < n; i++) {
                s_raw_snap[i] = g_sniff.raw[slot][i];
            }

            printf("raw seq=%lu kind=%s seen=%lu bits=%lu smpl=%lu "
                   "words=%lu nonzero=%lu (%lu samples, oldest first)\n",
                   (unsigned long)raw_seq,
                   kind ? "long" : "fail",
                   (unsigned long)rseen,
                   (unsigned long)rbits,
                   (unsigned long)rsmpl,
                   (unsigned long)n,
                   (unsigned long)rnz,
                   (unsigned long)(n * 8u));
            for (uint32_t i = 0; i < n; i++) {
                if ((i & 7u) == 0u) {
                    printf("R%04lX:", (unsigned long)i);
                }
                printf(" %08lX", (unsigned long)s_raw_snap[i]);
                if ((i & 7u) == 7u) {
                    printf("\n");
                }
            }
            printf("\n");
        }

        /* 已发布帧：打印统计值，并把整帧字节流打出来。
         * frame_no 由 Core 1 在数据写完后最后更新，所以看到它变化时，
         * cap_mosi/cap_miso 一定是完整的一帧。 */
        uint32_t frame = g_sniff.frame_no;
        if (frame != last_frame) {
            last_frame = frame;
            printf("frame %lu/%lu bits=%lu bytes=%lu smpl=%lu hin=%lu hix=%lu "
                   "hz=%lu us=%lu stall=%lu full=%lu last=%lub/%lus "
                   "csh=%lu tail=%08lX\n",
                   (unsigned long)frame,
                   (unsigned long)g_sniff.frames_seen,
                   (unsigned long)g_sniff.frame_bits,
                   (unsigned long)g_sniff.bytes_line,
                   (unsigned long)g_sniff.samples,
                   (unsigned long)g_sniff.sck_hi_min,
                   (unsigned long)g_sniff.sck_hi_max,
                   (unsigned long)g_sniff.sck_hz,
                   (unsigned long)g_sniff.elapsed_us,
                   (unsigned long)g_sniff.rxstall,
                   (unsigned long)g_sniff.frame_full,
                   (unsigned long)g_sniff.last_bits,
                   (unsigned long)g_sniff.last_samples,
                   (unsigned long)g_sniff.ring_cs_hi,
                   (unsigned long)g_sniff.ring_tail);

            /* 先把整帧拷贝出来，再慢慢打印（原因见 s_tx_mosi 的注释） */
            __dmb();
            uint32_t n = g_sniff.cap_bytes;
            if (n > SNIFF_DUMP_BYTES) {
                n = SNIFF_DUMP_BYTES;
            }
            for (uint32_t i = 0; i < n; i++) {
                s_tx_mosi[i] = g_sniff.cap_mosi[i];
                s_tx_miso[i] = g_sniff.cap_miso[i];
            }

            printf("dump %lu bytes\n", (unsigned long)n);
            dump_bytes("MOSI", s_tx_mosi, n);
            dump_bytes("MISO", s_tx_miso, n);
            printf("\n");
        }

        /* 20ms 一轮 ≈ 50Hz，满足屏幕刷新要求，也不会把 CPU 全占满 */
        sleep_ms(20);
    }
}
