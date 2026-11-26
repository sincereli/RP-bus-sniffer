#ifndef BOARD_H
#define BOARD_H

/*
 * 板级引脚定义（ATK-DNRP2350AM V1.0 开发板）
 *
 * 所有引脚都集中在这里，换板或改线只改这个文件。
 */

/* ---------------- 板载 1.14" ST7789 LCD（SPI1 硬件外设） ---------------- */
#define PIN_LCD_SCK     10
#define PIN_LCD_MOSI    11
#define PIN_LCD_CS       9
#define PIN_LCD_DC       8
#define PIN_LCD_BL      25

/* 背光由 PNP 三极管 S8550 驱动（3V3 -> 发射极 -> 集电极 -> R11 10R -> LEDA），
 * 基极经 R10 10K 接 GPIO25，R8 10K 上拉到 3V3。所以：拉低 = 点亮，拉高/悬空 = 熄灭。 */
#define LCD_BL_ON_LEVEL  0
#define LCD_BL_OFF_LEVEL 1

/* ---------------- 40MHz SPI 从机监听输入（高阻） ---------------- */
/* CS/SCK/MOSI/MISO 接 ESP32-S3 主机与 W25Q64 之间的总线，只做输入。
 *
 * 这四根线必须依次接在连续的 4 个 GPIO 上：PIO 用一条 `in pins,4`
 * 在一个周期里把四个脚同时锁存，IN_BASE 指向 CS。
 * 顺序写错或跳号会在编译期报 #error（见 sniffer.c）。 */
#define PIN_SNIFF_CS     17
#define PIN_SNIFF_SCK    18
#define PIN_SNIFF_MOSI   19
#define PIN_SNIFF_MISO   20

/* ---------------- 板载 TF 卡（SPI 模式，PIO 主机） ---------------- */
/* 原理图（ATK-DNRP2350AM V1.0，TF CARD 页）上卡槽是 SDIO 接线：
 *   SDIO_CLK = GPIO10   SDIO_CMD = GPIO11
 *   SDIO_D0  = GPIO12   SDIO_D1  = GPIO13
 *   SDIO_D2  = GPIO14   SDIO_D3  = GPIO15
 * SPI 模式只借其中 4 根，注意 D0/D3 的对应关系容易记反：
 *   SCK  = SDIO_CLK (10)   MOSI(卡的数据输入) = SDIO_CMD (11)
 *   MISO = SDIO_D0  (12)   CS               = SDIO_D3  (15)
 * SDIO_D1(13)/D2(14) 在 SPI 模式下不用，保持输入。
 *
 * 注意：SDIO_CLK/CMD 就是 GPIO10/11，和 LCD 的 SPI1 是同一对脚——
 * 板子把 LCD 与 TF 卡做成了共用总线。所以两者不能同时使用，固件里
 * 每次切换都要先释放一方（st7789_bus_release / sdcard_deinit）。 */
#define PIN_SD_CLK      10
#define PIN_SD_MOSI     11
#define PIN_SD_MISO     12
#define PIN_SD_CS       15

/* SPI 模式下不用的 SDIO_D1/D2，留着也要给个确定电平 */
#define PIN_SD_D1       13
#define PIN_SD_D2       14

/* ---------------- 系统参数 ---------------- */
/* SPI 监听用 PIO 过采样：状态机按 clk_sys/1 连续采样，每个 40MHz 位周期
 * 拿到 5 个采样点。RP2350A 官方标称 150MHz，200MHz 是常见稳定超频点，
 * 也是"40MHz SCK 每 bit 至少 5 点"的前提。 */
#define SNIFF_SYS_CLK_KHZ 200000

/* 小于这个 bit 数的帧按命令帧丢弃（8 bit = 1 字节）。
 * 128 = 16 字节：erase / read-ID 这类 4 字节命令帧被跳过，
 *       64 字节的读写帧（552 bit）保留；
 * 0 = 不过滤，每个 CS 帧都上屏（调试用，能看到"抓到 0 bit"这种帧）。 */
#define SNIFF_MIN_BITS 128u

/* ---------------- 原始采样落盘（诊断模式） ---------------- */
/* 1 = 不做软件解码，改为：CS 下降沿触发 -> 采满 CAP_WINDOW_US 微秒 ->
 *     停采样、把整个环形缓冲写进 TF 卡 -> 重新武装，共写 CAP_RECORDS 条。
 * 0 = 原来的实时解码模式（屏幕/串口显示解析结果）。
 *
 * 诊断模式的用途：把 200MHz 的原始采样点原样落盘，在 PC 上用
 * tools/capview.py 画波形，直接看毛刺、边沿、CS 抖动到底长什么样，
 * 不再依赖板上的解码逻辑去"猜"数据对不对。 */
#define SNIFF_CAPTURE_TO_SD  1

/* 触发后继续采样的时长（µs）。环形缓冲共 328µs，取 150µs 意味着
 * 每条记录里有约 150µs 的"触发后"波形和约 178µs 的"触发前"波形。 */
#define CAP_WINDOW_US        150u

/* 一次运行写多少条记录。每条 32KB（8192 个字），8 条约 256KB。 */
#define CAP_RECORDS          8u

/* 等 CS 下降沿的最长时间；超时就收尾，避免卡死在"等触发"上 */
#define CAP_TRIG_TIMEOUT_US  10000000u

/* ---------------- 记录模式 ---------------- */
/* CAP_MODE_WINDOW：一次性窗口（上一种模式）。CS 下降沿触发一次，停采样，
 *   把整个 328µs 环形缓冲写进卡，重新武装，共 CAP_RECORDS 条。
 *   简单，但两次记录之间的所有流量都漏掉了。
 *
 * CAP_MODE_STREAM：按帧连续记录（当前模式）。采样永不停：
 *   Core 1 在 CS 每次下降/上升时记下环形缓冲里的位置，把这一帧的原始
 *   采样字拷进 64KB 大缓冲；缓冲满就通过 FIFO 交给 Core 0 写卡，自己
 *   接着填另一块。总线空闲的那 99.96% 时间不占存储，所以能连续记录
 *   十几秒的全部事务，包括页编程和读数据的数据段。
 *
 * 存储量估算（ESP32 测试工程）：每 2 秒周期总线忙约 707µs，
 * 200MHz × 4bit = 100MB/s，折合约 71KB，平均 35KB/s。
 */
#define CAP_MODE_WINDOW      0
#define CAP_MODE_STREAM      1
#define CAP_MODE             CAP_MODE_STREAM

/* 连续记录的总时长（毫秒）。12 秒约等于 ESP32 的 6 个周期，
 * 每个周期里都有一次页编程和一次读数据，必然能录到数据段。 */
#define CAP_STREAM_MS        12000u

/* 缓冲块：8 块 × 16KB = 128KB。
 *
 * 为什么不是"2 块 × 64KB 乒乓"：Core 1 一旦等 Core 0 写完才能继续，等待
 * 期间描述符队列会积压，而环形缓冲只有 328µs，积压几毫秒后环早就绕过去
 * 了，取出来的全是几毫秒之后的新数据（这个坑踩过，见 sniffer.c 的
 * copy_frame）。改成多块 + 永不等：有空的块就交出去，没有就整块倒掉并
 * 计数。一次擦除故障期最多产生约 83KB 的帧数据，128KB 足够整段装下，
 * Core 0 有 1.9 秒的空闲时间慢慢写卡。 */
#define CAP_STREAM_BUF_BYTES (16u * 1024u)
#define CAP_STREAM_SLOTS     8u

#endif /* BOARD_H */
