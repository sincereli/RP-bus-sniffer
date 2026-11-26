#ifndef MW_POWER_H
#define MW_POWER_H

/*
 * 中间件层：电源与掉电保护（赛题 1.3 / F-13）
 *
 * 验收口径（见 资料/项目.md 抬头的"关键口径"）：
 *   "已 f_sync 的数据零丢失 + 文件系统可挂载"，
 *   表 1 字面的"零丢失"由 hold-up 电容 + 最后一帧测试保证。
 *
 * 掉电处理顺序：电压跌破阈值 -> 立即停采集 -> 停 UI 与外设 ->
 * 刷脏数据 -> 关闭文件 -> 允许断电。
 */

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PWR_OK = 0,
    PWR_LOW,        /* 低电告警：提示用户，正常收尾 */
    PWR_CRITICAL,   /* 即将掉电：立刻收尾，不再写新数据 */
} pwr_state_t;

void pwr_init(void);

/* 周期调用（主循环里），刷新内部状态。 */
pwr_state_t pwr_poll(void);

/* 电量百分比 0..100；无电量检测返回 0xFF。 */
uint8_t pwr_percent(void);

/* 电池电压（mV）；无检测返回 false。 */
bool pwr_battery_mv(uint16_t *mv);

/*
 * 严重欠压回调。在中断上下文触发，实现里只置标志，真正的收尾由主循环
 * 完成——不要在回调里直接写卡。
 */
typedef void (*pwr_critical_cb_t)(void);
void pwr_set_critical_handler(pwr_critical_cb_t cb);

#endif /* MW_POWER_H */
