#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>

/*
 * TF 卡驱动（SPI 模式，PIO 主机）。
 *
 * 板载卡槽是 4bit SDIO 接线，这里只用其中 4 根线走 SPI 模式：
 *   CLK = GPIO10、CMD = MOSI、D0 = MISO、D3 = CS。
 * 注意 GPIO10/11 同时接着 LCD 的 SCK/MOSI，所以使用本驱动之前必须先
 * 调用 st7789_bus_release() 把 LCD 的 SPI1 从这两个脚上释放掉。
 */

typedef struct {
    bool     block_addressing;  /* true = SDHC/SDXC，地址按 512 字节块给 */
    bool     v2;                /* true = 支持 CMD8 的 SD v2 卡 */
    uint32_t err_stage;         /* 初始化失败时的阶段号，0 表示成功 */
    uint32_t r7;                /* CMD8 的 R7 响应（电压/回声校验） */
    uint32_t ocr;               /* CMD58 读到的 OCR */
} sdcard_info_t;

/*
 * 初始化卡：上电时钟 80 拍、CMD0/CMD8/ACMD41/CMD58，成功后再把时钟提到
 * 12.5MHz。返回 false 时 info->err_stage 指出卡在哪一步：
 *   1 = CMD0 无响应      2 = CMD8 响应异常
 *   3 = ACMD41 超时      4 = CMD58 异常
 */
bool sdcard_init(sdcard_info_t *info);

/* 释放总线：停掉 PIO 状态机，把 4 根脚交回 SIO 输入，之后 LCD 可以重新接管 */
void sdcard_deinit(void);

/* 按扇区读写，count 可以大于 1；内部逐块发 CMD17/CMD24 */
bool sdcard_read_blocks(uint32_t lba, uint8_t *buf, uint32_t count);
bool sdcard_write_blocks(uint32_t lba, const uint8_t *buf, uint32_t count);

#endif /* SDCARD_H */
