#ifndef MW_STORAGE_H
#define MW_STORAGE_H

/*
 * 中间件层：TF 卡存储
 *
 * 作为一个 log sink 注册到 middleware/log.h。对上层暴露的是
 * "能不能录、还剩多少空间、格式化"，不暴露 FatFs 细节。
 *
 * 赛题要求（1.3 / F-12）：
 *   - 按日期自动生成目录：/YYYYMMDD/
 *   - 文件名带总线名与时分秒：SPI_143005.log
 *   - 掉电不损坏已有日志：周期性 f_sync()，掉电预警时立即 f_close()
 */

#include <stdint.h>
#include <stdbool.h>
#include "middleware/log.h"
#include "middleware/timebase.h"

typedef struct {
    const char *bus_name;    /* "SPI" / "I2C" / "UART" / "SWD" */
    rtc_time_t  start_time;  /* 用于生成目录与文件名 */
    uint32_t    max_bytes;   /* 单文件上限，0 = 不限（午夜切分另行处理） */
} storage_session_t;

bool storage_init(void);          /* 探测并挂载卡 */
bool storage_ready(void);
bool storage_format(void);        /* 格式化（破坏性，调用前必须由应用层确认） */

/* 剩余空间（字节）。未挂载返回 0。 */
uint64_t storage_free_bytes(void);

/* 把存储注册进日志系统：log_attach_sink(storage_sink()) */
const log_sink_t *storage_sink(void);

/* 掉电前调用：立刻刷脏数据并关闭当前文件。可在中断上下文调用，
 * 实现上只做"置标志 + 唤醒"，真正的写操作由主循环完成。 */
void storage_emergency_close(void);

#endif /* MW_STORAGE_H */
