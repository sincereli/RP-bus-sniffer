#ifndef APP_APP_H
#define APP_APP_H

/*
 * 应用层：入口与主循环
 *
 * 本层只做三件事：把用户操作变成模式请求、把中间件的数据变成屏幕内容、
 * 把上位机命令交给对应模块。它不直接碰任何总线硬件，也不解析任何协议
 * 负载 —— 那两件事分别属于协议层和中间件层。
 *
 * 应用层内部按职责再分，头文件在实现阶段补齐：
 *   ui.c      屏幕渲染（读 log_stats / log_last_summary / mode_context）
 *   menu.c    菜单树，协议清单来自 proto_count()/proto_at()
 *   host.c    上位机命令会话，命令号分发给协议模块
 */

#include <stdint.h>
#include <stdbool.h>

/* 上电初始化顺序（必须遵守）：
 *   bsp_init -> tb_init -> pwr_init -> storage_init -> usbdev_init
 *   -> cfg_load(config.txt) -> proto_init -> app 自身状态
 */
void app_init(void);

/* Core 0 主循环里周期调用。内部按需刷新 UI（>=30Hz）、扫描摇杆、
 * 处理 USB 命令、调度模式切换。不得长时间阻塞。 */
void app_tick(void);

/* 实时性要求：屏幕刷新率 >= 30Hz（赛题表 1），所以 app_tick 的空转
 * 周期应远小于 33ms；采集数据的搬运不在本函数里，由 Core 1 负责。 */
#define APP_TICK_MS 5u

#endif /* APP_APP_H */
