#ifndef HAL_BUS_H
#define HAL_BUS_H

/*
 * 硬件抽象层：总线线路与角色
 *
 * 这一层只描述"总线长什么样、现在是什么角色"，不碰具体外设寄存器。
 * 引脚号来自 BSP（bsp/bsp.h），PIO/DMA 资源来自 hal/pio_alloc.h。
 *
 * 不依赖任何 Pico SDK 头文件：引脚用 int8_t，未使用填 -1。
 */

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    BUS_ID_SPI = 0,
    BUS_ID_I2C,
    BUS_ID_UART,
    BUS_ID_SWD,
    BUS_ID_MAX,
} bus_id_t;

typedef enum {
    BUS_ROLE_HIZ = 0,    /* 全部高阻，不接任何信号（安全状态） */
    BUS_ROLE_SNIFF,      /* 从机监听：只输入，绝不影响被测总线 */
    BUS_ROLE_MASTER,     /* 主机读写：输出使能，电平转换按目标电平方位 */
} bus_role_t;

/* 一条总线的引脚分配。未使用的字段填 -1。
 * SPI 监听要求 cs/sck/mosi/miso 是连续 4 个 GPIO（PIO 一条 in pins,4 锁存）。 */
typedef struct {
    int8_t cs;
    int8_t sck;
    int8_t mosi;
    int8_t miso;
    int8_t sda;
    int8_t scl;
    int8_t tx;
    int8_t rx;
    int8_t swdio;
    int8_t swclk;
} bus_lines_t;

/* 电平方位：赛题要求兼容 3.3V/5V */
typedef enum {
    BUS_LEVEL_3V3 = 0,
    BUS_LEVEL_5V0,
} bus_level_t;

/*
 * 把一条总线切到指定角色：
 *   1. 先把所有相关引脚置高阻（防两端同时驱动）
 *   2. 配置方向与电平转换使能
 *   3. 交给 BSP 做硬件复用器切换
 * 返回 false 表示该总线不支持这个角色（例如 SWD 没有监听模式）。
 */
bool bus_setup(bus_id_t id, bus_role_t role, bus_level_t level);

/* 恢复高阻。切换总线、进入低功耗、掉电保护时调用。 */
void bus_release(bus_id_t id);

/* 查询当前状态 */
bus_role_t bus_current_role(bus_id_t id);
bool       bus_is_acquired(bus_id_t id);

#endif /* HAL_BUS_H */
