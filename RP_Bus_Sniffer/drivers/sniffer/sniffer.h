#ifndef SNIFFER_H
#define SNIFFER_H

#include <stdint.h>
#include "board.h"

/*
 * Core 1 采集引擎与 Core 0 之间的共享结果区。
 *
 * 这是两个核唯一的"结果通道"（FIFO 只传"有新帧"的通知码）：
 *
 *   写方 Core 1 的顺序：
 *       填完所有数据字段 -> __dmb() -> 最后写 state / frame_no / frames_seen
 *   读方 Core 0 的顺序：
 *       __dmb() -> 读标志字段 -> 判断变化 -> 再读数据字段
 *
 *   __dmb()（数据存储屏障）保证屏障之前的写在屏障之后的写之前对所有核可见。
 *   因此 Core 0 只要看到标志变化，读到的数据就一定是完整的一帧。
 *
 *   所有字段都声明为 volatile：它们会被另一个核异步修改，编译器不能把
 *   读取结果缓存进寄存器，也不能重排或删除这些访问。
 */

/* 采集状态。屏幕/串口上的 STATE 显示的就是它。
 * 注意：被过滤掉的短命令帧不会改变已发布状态，只会在诊断字段里出现。 */
typedef enum {
    SNIFF_IDLE = 0,      /* 上电初始值，尚未启动采集 */
    SNIFF_ARMED,         /* 采样已就绪，正在等 CS 拉低 */
    SNIFF_CAPTURING,     /* CS 已拉低，正在等这一帧结束 */
    SNIFF_DONE,          /* 上一帧正常收完并已发布 */
    SNIFF_TIMEOUT,       /* 上一帧没有正常结束（CS 一直没回高） */
    SNIFF_ERROR,         /* 初始化自检失败，引擎已停止 */
} sniff_state_t;

/* 每帧最多保留多少字节给串口/上位机查看。
 * 解析上限是 20000 个采样点（20µs/100µs 内 4000 bit = 500 字节），
 * 取 512 足够覆盖；超出部分只截断转储，位计数与哈希仍完整统计。 */
#define SNIFF_DUMP_BYTES 512u

/* 解码失败时，拷给 Core 0 的原始采样字数（每字 8 个采样点）。
 * 1024 字 = 8192 个采样点 = 41µs，足够覆盖一帧 14µs 外加等待时间。 */
#define SNIFF_RAW_WORDS 1024u

/* ---- 原始采样落盘（诊断模式，见 board.h 的 SNIFF_CAPTURE_TO_SD）----
 * 这条通路不做任何解码：触发一次就把整个环形缓冲原样写进 TF 卡，
 * 由 PC 端 tools/capview.py 画波形。状态机：
 *   SD_INIT -> ARMED -> WINDOW -> WRITING -> ARMED ... -> DONE
 *                  ^                          |
 *                  +--------------------------+
 */
typedef enum {
    CAP_OFF = 0,        /* 未启用 */
    CAP_SD_INIT,        /* 正在初始化卡/挂载/建文件 */
    CAP_ARMED,          /* 采样运行，等 CS 下降沿 */
    CAP_WINDOW,         /* 已触发，正在采触发后的固定窗口 */
    CAP_WRITING,        /* 采样已停，正在把环形缓冲写完 */
    CAP_DONE,           /* 全部记录写完，文件已关闭 */
    CAP_ERROR,          /* cap_err 里是失败阶段 */
} cap_state_t;

/* 单个 CS 帧的处理结果快照（诊断用）。
 * Core 1 每处理一帧填一份，Core 0 只读快照，避免逐字段读到不同帧的值。
 * 双缓冲：Core 1 交替写两份，写完整份后 diag_seq 才加一。 */
typedef struct {
    uint32_t seen;      /* 这是第几个检测到的 CS 帧 */
    uint32_t decoded;   /* 1 = 找到了稳定 CS 下降沿并解出帧 */
    uint32_t bits;      /* 解出的 bit 数 */
    uint32_t samples;   /* 帧在采样流里的长度（采样点数） */
    uint32_t csh;       /* 最近 4096 个采样点里 CS 为高的个数 */
    uint32_t tail;      /* 最近 8 个采样点的原始 nibble */
    uint32_t head;      /* 解码时 DMA 写指针（采样点位置） */
    uint32_t waddr;     /* 解码时 DMA 写地址原始值 */
    uint32_t stall;     /* PIO RX FIFO 是否溢出过 */
    uint32_t elapsed;   /* CS 下降沿到帧结束的微秒数 */
    uint32_t full;      /* 1 = 帧长超过解析上限被截断 */

    /* ---- 帧尾判定与解码失败的证据（用来区分"真结束"和"CS 毛刺"）---- */
    uint32_t end_run;   /* 判定帧尾时 CS 连续为高的采样点数；0 = 撞写指针/到上限 */
    uint32_t hit_head;  /* 1 = 解析追到 DMA 写指针，帧尾尚未写进环形缓冲 */
    uint32_t saw;       /* 1 = 主循环亲眼看到 CS 回高，elapsed 是实测值 */
    uint32_t cand;      /* 回溯扫描中遇到的 CS 下降沿候选数 */
    uint32_t max_low;   /* 候选中"下降沿之后 32 点里 CS 低"的最大个数 */
    uint32_t max_high;  /* 候选中"下降沿之前 32 点里 CS 高"的最大个数 */
} sniff_diag_t;

/* ---- 按帧连续记录（CAP_MODE_STREAM）----
 *
 * 记录格式（RPSTREAM.BIN，小端）：
 *
 *   文件头 24 字节
 *     offset 0  magic       'RPSF' = 0x46535052
 *     offset 4  version     u16 = 1
 *     offset 6  hdr_bytes   u16 = 24
 *     offset 8  sample_hz   采样率（实测 clk_sys）
 *     offset 12 clk_khz     实测 clk_sys（kHz），和上面同源，冗余一份便于核对
 *     offset 16 reserved    0
 *     offset 20 reserved    0
 *
 *   之后是若干"帧记录"，每条 8 字节头 + 原始采样字：
 *     offset 0  t_us    帧开始时刻，相对采集起点的微秒数
 *     offset 4  words   后面跟的 32bit 字数
 *     offset 6  flags   bit0 = 截断（没在环里找到 CS 上升沿，按上限截断）
 *     offset 7  pad     0
 *     offset 8  words*4 字节原始采样字
 *
 * 字的位布局与 RPCAP.BIN 完全一致：bit[4k+0]=CS  +1=SCK  +2=MOSI  +3=MISO，
 * k=0 是该字里最早的采样点。所以 PC 端两条通路可以共用同一套解包代码。
 */
#define CAP_STREAM_MAGIC   0x46535052u   /* 'RPSF' 小端 */
#define CAP_STREAM_HDR     24u
#define CAP_STREAM_RECHDR  8u

typedef enum {
    CAP_SLOT_FREE = 0,      /* 可以写 */
    CAP_SLOT_FILLING,       /* Core 1 正在往里写 */
    CAP_SLOT_READY,         /* Core 1 写满/收尾，等 Core 0 落盘 */
} cap_slot_state_t;

typedef struct {
    volatile uint32_t state;    /* cap_slot_state_t */
    volatile uint32_t bytes;    /* 有效字节数 */
    volatile uint32_t frames;   /* 这块里有多少个 CS 帧 */
} cap_slot_t;

#if SNIFF_CAPTURE_TO_SD && (CAP_MODE == CAP_MODE_STREAM)
/* 乒乓缓冲。定义在 sniffer.c；Core 1 只写正在填充的那一块，
 * Core 0 只读已经置成 READY 的那一块，两者不会同时碰同一块。 */
extern uint8_t    g_cap_buf[CAP_STREAM_SLOTS][CAP_STREAM_BUF_BYTES];
extern cap_slot_t g_cap_slot[CAP_STREAM_SLOTS];
#endif

typedef struct {
    /* ---- 状态标志 ---- */
    volatile uint32_t state;         /* sniff_state_t，屏幕上的 STATE */

    /* ---- 本帧结果（只在发布帧时整体刷新）---- */
    volatile uint32_t frame_bits;    /* 帧内 SCK 上升沿个数 = 每条线的 bit 数 */
    volatile uint32_t bytes_line;    /* frame_bits / 8（完整字节数） */
    volatile uint32_t samples;       /* 帧内 CS 低期间的 200MHz 采样点数 */

    volatile uint32_t frame_no;      /* 已发布的帧数（从 1 开始），Core 0 的"有新帧"信号 */
    volatile uint32_t frames_seen;   /* 检测到的 CS 帧总数（含被过滤的短帧） */
    volatile uint32_t frame_ok;      /* 1 = CS 正常回高，帧完整 */
    volatile uint32_t frame_full;    /* 1 = 帧长超过软件解析上限，被截断 */

    /* ---- 最近一次解码尝试的结果（包括被过滤的短帧）----
     * 只用于诊断：它和上面"已发布的结果"分开存，短帧/毛刺不会把屏幕上
     * 已有的好数据冲掉。
     *   last_decoded = 0 表示环形缓冲里没找到稳定的 CS 下降沿 */
    volatile uint32_t last_bits;
    volatile uint32_t last_samples;
    volatile uint32_t last_decoded;

    /* SCK 高电平宽度，单位是 5ns 采样点。
     * 40MHz、50% 占空比时每个高电平约 12.5ns，即 2~3 个采样点。
     *   min = 1  -> 有的高电平只有一个采样点宽，边沿已经明显退化
     *   min = 0  -> 这一帧量不到完整高电平
     *   max 很大 -> SCK 有较长的高电平（帧间空闲或握手等待） */
    volatile uint32_t sck_hi_min;
    volatile uint32_t sck_hi_max;

    volatile uint32_t elapsed_us;    /* CS 下降到 CS 回升的时间（XOSC 时基） */
    volatile uint32_t rxstall;       /* 1 = PIO RX FIFO 溢出过：DMA 没跟上 */

    /* ---- 环形缓冲健康状态（每次解码都会刷新，用于定位问题）----
     *   ring_cs_hi：最近 4096 个采样点里 CS 为高的个数（空闲时接近 4096）；
     *   ring_tail ：最近 8 个采样点的原始 4bit 字段，每个 nibble 为
     *               [MISO MOSI SCK CS]，空闲时应是 1 或 9 重复。
     *   全 0 说明 DMA 根本没往环里写；恒为 4096 / 0x11111111 说明 CS 采样恒高。
     *   ring_head：DMA 下一个写入的采样点位置，两次事件之间必须一直在变。 */
    volatile uint32_t ring_cs_hi;
    volatile uint32_t ring_tail;
    volatile uint32_t ring_head;
    volatile uint32_t dma_waddr;     /* DMA 写地址原始值（调试用） */

    /* ---- 已发布帧的前 8 个字节，供屏幕直接显示 ---- */
    volatile uint32_t first_mosi[8];
    volatile uint32_t first_miso[8];

    /* ---- 整帧 FNV-1a 哈希：同一地址连读时每帧应完全一致 ---- */
    volatile uint32_t hash_mosi;
    volatile uint32_t hash_miso;

    /* ---- 已发布帧的完整字节流（最多 SNIFF_DUMP_BYTES 字节）----
     * Core 1 只在发布帧时更新，且更新发生在写 frame_no 之前；
     * Core 0 看到新的 frame_no 时，这里一定是完整的一帧。 */
    volatile uint32_t cap_bytes;     /* 有效字节数 */
    volatile uint8_t cap_mosi[SNIFF_DUMP_BYTES];
    volatile uint8_t cap_miso[SNIFF_DUMP_BYTES];

    /* ---- 原始采样转储（解码失败前 2 次 + 长帧第 1 次）----
     * raw_seq 变化表示有新的片段；Core 0 据此把原始字打到串口。
     * 用来判断问题出在 PIO 采样、DMA 写入还是软件解析。
     *
     * 双缓冲：Core 1 写到 raw[raw_slot]，写完整段后再更新 raw_seq；
     * Core 0 按 raw_slot 读对应那一份。否则 Core 0 一边慢慢打印、Core 1
     * 一边覆盖 raw[]，日志里会混进两次不同转储的内容（之前 dump 和
     * evt 对不上就是这个原因）。 */
    volatile uint32_t raw_seq;
    volatile uint32_t raw_slot;      /* 本片段在 raw[] 里的槽位（0/1） */
    volatile uint32_t raw_count;
    volatile uint32_t raw_nonzero;   /* 片段里非零字的个数（0 = 环是空的） */
    volatile uint32_t raw_kind;      /* 0 = 解码失败转储，1 = 长帧转储 */
    volatile uint32_t raw_bits;      /* 转储时该帧解出的 bit 数（失败时为 0） */
    volatile uint32_t raw_samples;   /* 转储时该帧的采样点数（失败时为 0） */
    volatile uint32_t raw_seen;      /* 转储对应的是第几个 CS 事件 */
    volatile uint32_t raw[2][SNIFF_RAW_WORDS];

    /* ---- 每帧诊断快照（双缓冲）----
     * Core 1 处理完一帧后，先填 diag[idx]，再用 __dmb() 保证写完，
     * 最后才让 diag_seq 加一。Core 0 只按 diag_seq 的变化读快照，
     * 这样一次打印里的所有字段一定来自同一个 CS 帧，不会互相串帧。
     * 两份交替使用：Core 1 写下一帧时改的是另一份，Core 0 慢慢打印
     * 也不会被改到。 */
    volatile uint32_t diag_seq;
    volatile sniff_diag_t diag[2];

    /* ---- 原始采样落盘状态 ---- */
    volatile uint32_t cap_state;      /* cap_state_t */
    volatile uint32_t cap_index;      /* 已写完的记录条数 */
    volatile uint32_t cap_total;      /* 计划写入的条数 */
    volatile uint32_t cap_err;        /* 失败阶段：1~4 = SD 初始化，10 = 挂载，
                                       * 11 = 建文件，12 = 写文件，13 = 等触发超时 */
    volatile uint32_t cap_file_bytes; /* 文件总字节数（收尾时填写） */
    volatile uint32_t cap_last_ms;    /* 最近一条记录从触发到 f_sync 的毫秒数 */
    volatile uint32_t cap_trig_word;  /* 触发时刻 DMA 写指针在环里的字序号 */
    volatile uint32_t cap_stop_word;  /* 停止采样时刻的写指针（记录起点） */
    volatile uint32_t cap_sample_hz;  /* 采样率 = 实测 clk_sys */

    /* ---- 按帧连续记录的统计（CAP_MODE_STREAM）---- */
    volatile uint32_t cap_mode;        /* CAP_MODE_WINDOW / CAP_MODE_STREAM */
    volatile uint32_t cap_frames;      /* 已写进文件的 CS 帧数 */
    volatile uint32_t cap_frame_maxw;  /* 单帧最大字数（诊断：正常读帧约 340） */
    volatile uint32_t cap_glitch;      /* 丢掉的高频毛刺（CS 低 < 2 个字） */
    volatile uint32_t cap_lost;        /* 因缓冲满/环被覆盖而丢掉的帧 */
    volatile uint32_t cap_stale;       /* 描述符太老、环已被覆盖而丢掉的帧 */
    volatile uint32_t cap_slotdrop;    /* 没有空闲缓冲块，整块倒掉的次数 */
    volatile uint32_t cap_qdrop;       /* 中断队列溢出丢掉的帧 */
    volatile uint32_t cap_backw;       /* 帧起点回溯距离的最大值（字） */
    volatile uint32_t cap_stream_done; /* 1 = Core 1 已收尾，Core 0 写完剩余缓冲即可退出 */

    /* ---- 帧起点定位的分支统计（诊断外推模型是否可靠）---- */
    volatile uint32_t cap_fall_near;   /* 外推点 ±32 字内找到下降沿 */
    volatile uint32_t cap_fall_wide;   /* 外推点 ±128 字内找到 */
    volatile uint32_t cap_fall_boot;   /* 外推落空，退回中断写指针窗口找到 */
    volatile uint32_t cap_fall_miss;   /* 全部落空，这一帧按截断处理 */
    volatile uint32_t cap_pred_err;    /* 外推点与找到的下降沿的最大偏差（字） */
    volatile uint32_t cap_age_max;     /* 描述符被消费时的最大年龄（µs） */

    /* 最近一次"外推落空"的现场，用来定位模型为什么偏 */
    volatile uint32_t cap_miss_t;      /* 描述符时间戳（µs） */
    volatile uint32_t cap_miss_startw; /* 中断里记下的写指针位置（环内字序号） */
    volatile uint32_t cap_miss_expect; /* 按时间外推出的位置（环内字序号） */

    /* ---- 频率信息 ---- */
    volatile uint32_t sck_hz;        /* 由本帧 bit 数和持续时间估算的 SCK 频率 */
    volatile uint32_t clk_sys_hz;    /* 时钟树配置值（不是实测） */
    volatile uint32_t clk_meas_khz;  /* 用 XOSC 时基实测的 clk_sys（0 = 未测） */
} sniff_shared_t;

/* 全局共享实例。定义在 sniffer.c，Core 0/1 都直接访问。 */
extern sniff_shared_t g_sniff;

/*
 * 采集引擎入口，作为 multicore_launch_core1() 的回调在 Core 1 上运行。
 *
 * 它会：
 *   1. 配置四根监听脚、PIO 过采样和 DMA 环形缓冲；
 *   2. 实测 clk_sys；
 *   3. 进入永不返回的循环：等 CS 帧 -> 在环形缓冲里解码 -> 发布到 g_sniff。
 * 不接收参数，也不返回。
 */
void sniffer_core1_entry(void);

#endif /* SNIFFER_H */
