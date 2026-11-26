#ifndef MW_LOG_H
#define MW_LOG_H

/*
 * 中间件层：日志扇出
 *
 * 一条记录从协议引擎进来，经编码器变成统一记录后，同时送进若干 sink：
 *   - storage sink：写 TF 卡
 *   - usb sink：实时上传上位机
 *   - 屏幕：只取 summarize() 的一行摘要，不算 sink
 *
 * sink 是注册进来的，所以"往哪里送"也能扩展（例如以后加板载 Flash
 * 掉电保险箱，加一个 sink 即可，logging 主体不动）。
 */

#include <stdint.h>
#include <stdbool.h>
#include "record.h"

/* ---------------- sink 契约 ----------------
 * 三个函数都在 log_submit() 的调用栈里被同步调用。
 * 注意：**不得阻塞**。写卡慢的时候要自己排队或丢弃，不能把采集回路拖住。
 */
typedef struct {
    const char *name;

    /* 会话开始。参数是会话信息（文件路径、采样配置等），由 sink 自己解释 */
    bool (*open)(const void *session_cfg);

    /* 接收一条**已序列化**的记录（布局见 record.h 的"线上序列化格式"） */
    void (*write)(const uint8_t *data, uint32_t len);

    /* 周期性刷盘（TF 卡走 f_sync）；无此需求的 sink 可留 NULL */
    void (*sync)(void);

    /* 会话结束：写完残留数据、关文件 */
    void (*close)(void);
} log_sink_t;

/* ---------------- 注册与生命周期 ---------------- */

/* 注册一个 sink。必须在 log_open() 之前调用。最多 LOG_SINK_MAX 个。 */
bool log_attach_sink(const log_sink_t *sink);

/* 打开会话：清空统计，依次调用各 sink 的 open()。 */
bool log_open(const void *session_cfg);

/* 会话统计快照，供屏幕和上位机查询 */
typedef struct {
    uint32_t records;        /* 已发出的记录数 */
    uint32_t bytes;          /* 已序列化字节数 */
    uint32_t dropped;        /* 因缓冲不足被丢掉的记录数 */
    uint32_t encode_fail;    /* 编码器返回 0 或查不到编码器的次数 */
} log_stats_t;

void log_stats(log_stats_t *out);

/* 结束会话：调用各 sink 的 close()。 */
void log_close(void);

/* ---------------- 记录投递 ----------------
 * 协议引擎每产出一条原生记录就调用一次。
 *   proto_id：引擎所属协议
 *   native_rec：协议私有的记录结构体指针
 *
 * 中间件按 proto_id 查编码器 -> encode() -> 序列化 -> 扇出到各 sink。
 * 返回 false 表示没有可用编码器（说明协议模块忘了注册）。
 *
 * **不阻塞、不分配内存**：编码缓冲区是中间件内部的一块静态缓冲。
 */
bool log_emit(uint8_t proto_id, const void *native_rec);

/* ---------------- 给屏幕的摘要 ----------------
 * 返回最近一条记录的摘要行（由对应编码器的 summarize() 生成）。
 * 没有新记录时返回 false，UI 可以继续显示上一行。
 */
bool log_last_summary(char *line, uint16_t cap);

#define LOG_SINK_MAX 4u

#endif /* MW_LOG_H */
