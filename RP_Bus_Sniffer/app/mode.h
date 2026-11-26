#ifndef APP_MODE_H
#define APP_MODE_H

/*
 * 应用层：工作模式状态机
 *
 * 这是唯一知道"当前该用哪个协议、哪个角色"的地方。协议引擎层不知道自己
 * 被谁调用，中间件也不知道当前是什么模式 —— 这样加协议不需要改它们。
 *
 * 切换模式时必须按固定顺序做，否则会两个引擎抢同一组 PIO/DMA 或两个
 * 方向同时驱动总线：
 *     旧引擎 close() -> bus_release() -> log_close()
 *     -> bus_setup(新) -> 新引擎 open() -> log_open()
 */

#include <stdint.h>
#include <stdbool.h>
#include "middleware/record.h"

typedef enum {
    MODE_IDLE = 0,      /* 上电后：显示状态栏，等待用户选择 */
    MODE_SNIFF,         /* 从机监听抓包并落盘 */
    MODE_MASTER,        /* 主机读写外设（离线寄存器调试） */
    MODE_BURN,          /* SWD 烧录 PY32F003 */
    MODE_BRIDGE,        /* USB 转多接口透传 */
    MODE_REPLAY,        /* 把 SD 卡历史日志按倍速回放到上位机 */
} app_mode_t;

/* 请求切换模式。返回 false 表示这个模式当前不可用
 * （例如卡没插不能进 SNIFF，或目标协议没有主机能力）。 */
bool mode_request(app_mode_t m);

/* 立即执行已请求的切换（在安全点调用，不在中断里）。 */
void mode_apply_pending(void);

app_mode_t mode_current(void);

/*
 * 当前模式的上下文。SNIFF/MASTER 用 proto_id 指明协议；
 * 具体参数由对应协议的配置结构体携带（见 protocol/engine.h）。
 */
typedef struct {
    app_mode_t mode;
    uint8_t    proto_id;    /* MODE_SNIFF / MODE_MASTER 时有效 */
    bool       recording;   /* 是否正在落盘 */
    uint64_t   started_us;  /* 本次会话开始时刻（tb_mono_us） */
} mode_ctx_t;

const mode_ctx_t *mode_context(void);

/* 模式名（给屏幕和日志用）："IDLE" / "SPI-SNIFF" … */
const char *mode_name(app_mode_t m, uint8_t proto_id);

#endif /* APP_MODE_H */
