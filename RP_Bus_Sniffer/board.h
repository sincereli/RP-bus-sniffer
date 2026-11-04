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

#endif /* BOARD_H */
