#ifndef BSP_H
#define BSP_H

/*
 * 板级资源包（与 hal/ 同属第三层）
 *
 * hal/ 是与板无关的抽象接口，bsp/ 是它在某块具体板子上的落地：
 * 引脚从哪儿来、复用器怎么切、电平转换怎么使能、电池电压怎么采。
 *
 * **换板只改这一层**（引脚宏目前仍在根目录 board.h，迁移时整体移到
 * bsp/ 下；迁移完成前不要两处各写一份引脚）。
 */

#include <stdint.h>
#include <stdbool.h>
#include "hal/bus.h"

/* 上电初始化：时钟树、GPIO 默认状态、复用器置安全位。
 * 必须在其它任何模块之前调用。 */
bool bsp_init(void);

/* 取某条总线的引脚分配。返回 NULL 表示该板上没有这条总线。 */
const bus_lines_t *bsp_bus_lines(bus_id_t id);

/* 硬件复用器：把某条总线接到外部接口，并设定电平方位。
 * 返回 false 表示该板没有这条通路。 */
bool bsp_mux_select(bus_id_t id, bus_role_t role, bus_level_t level);

/* 把复用器全部断开（进入高阻安全态） */
void bsp_mux_release(void);

/* I2C 上拉电阻可切换（赛题 1.2：IIC/SPI 兼容 3.3V/5V） */
bool bsp_i2c_pullup(bool enable);

/* 板载屏幕背光 */
void bsp_display_backlight(bool on);

/* 电池电压（mV）。返回 false 表示本板没有电量检测。 */
bool bsp_battery_mv(uint16_t *mv);

/* 烧录状态指示灯（赛题 1.1：待机/闪烁/成功/失败） */
typedef enum {
    LED_OFF = 0,
    LED_IDLE,
    LED_BUSY,
    LED_OK,
    LED_FAIL,
} bsp_led_t;

void bsp_burn_led(bsp_led_t state);

#endif /* BSP_H */
