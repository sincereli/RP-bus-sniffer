#ifndef MW_TIMEBASE_H
#define MW_TIMEBASE_H

/*
 * 中间件层：时间基
 *
 * 赛题要求每条记录带时间戳、日志按日期分目录（1.3），上位机要按时间戳
 * 1x/2x/10x 回放（表 1）。
 *
 * 时间来源有两个：
 *   - 64 位硬件微秒定时器：单调、分辨率 1µs，上电起算，用于记录内的时间差；
 *   - 外置 RTC：绝对日历时间，用于文件命名与跨设备对齐。
 * 两者在 RTC 每秒中断时做一次同步，把"微秒计数"锚到"绝对时间"上。
 *
 * 跨核访问：Core 1 在采集回路里只读，同步点由 Core 0 在中断里写。
 * 实现时必须用数据屏障或 spinlock 保护，避免读到半更新的 64 位数（撕裂）。
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t year;    /* 完整年份，如 2026 */
    uint8_t  month;   /* 1..12 */
    uint8_t  day;     /* 1..31 */
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} rtc_time_t;

bool tb_init(void);

/* 上电起的单调微秒数。采集回路里可以直接高频调用。 */
uint64_t tb_mono_us(void);

/* 会话时间戳：以 log_open() 的时刻为 0 的微秒数。
 * 记录里的 t_us 用这个，方便 PC 端按相对时间画波形。 */
uint64_t tb_session_us(void);

/* 绝对时间（Unix 纪元微秒）。RTC 尚未同步时返回 0。 */
uint64_t tb_epoch_us(void);

bool tb_rtc_read(rtc_time_t *t);
bool tb_rtc_set(const rtc_time_t *t);

/* 外置 RTC 的 1Hz 秒中断里调用：锁存当前微秒值，刷新绝对时间基准。 */
void tb_rtc_second_isr(void);

#endif /* MW_TIMEBASE_H */
