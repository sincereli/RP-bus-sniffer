#ifndef HAL_PIO_ALLOC_H
#define HAL_PIO_ALLOC_H

/*
 * 硬件抽象层：PIO 状态机与 DMA 通道的资源分配
 *
 * RP2350 有 3 个 PIO，每个 4 个状态机；DMA 有 16 个通道。多个协议引擎
 * （SPI 监听、I2C 监听、UART 过采样、SWD）会同时存在，必须统一分配，
 * 不能让各模块自己写死 PIO0/SM0 —— 否则加一个协议就可能和已有引擎撞车。
 *
 * 这一层只做"谁用哪个 PIO/SM/DMA"的记账，不配置具体寄存器。
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int8_t pio;    /* 0..2，-1 = 未占用 */
    int8_t sm;     /* 0..3，-1 = 未占用 */
    int8_t dma;    /* 0..15，-1 = 未占用（不需要 DMA 时保持 -1） */
} hal_pio_t;

/*
 * 申请一个能装下给定程序的状态机（含所需指令槽），并可选申请一个 DMA 通道。
 *   prog：PIO 程序描述（pio_program_t，用 void* 是为了本头文件不依赖 SDK）
 *   need_dma：true 时同时申请一个空闲 DMA 通道
 * 成功返回 true，r 里填好实际占用的资源。失败说明资源已用尽。
 */
bool hal_pio_claim(hal_pio_t *r, const void *prog, bool need_dma);

/* 归还全部资源。可以重复调用。 */
void hal_pio_release(hal_pio_t *r);

/* 资源使用情况，供状态查询/屏幕诊断 */
typedef struct {
    uint8_t sm_used[3];     /* 每个 PIO 已用状态机数 */
    uint8_t dma_used;       /* 已用 DMA 通道数 */
} hal_pio_usage_t;

void hal_pio_usage(hal_pio_usage_t *out);

#endif /* HAL_PIO_ALLOC_H */
