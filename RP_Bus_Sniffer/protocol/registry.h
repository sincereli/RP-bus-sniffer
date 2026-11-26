#ifndef PROTO_REGISTRY_H
#define PROTO_REGISTRY_H

/*
 * 协议引擎层：协议注册表
 *
 * 应用层的菜单不是硬编码的协议列表，而是遍历这张表生成的——否则加一台
 * 新协议就得同时改菜单，等于把"不改上层"的口子漏在应用层。
 *
 * 注册发生在 proto_init()：它逐个调用内置模块的注册函数。
 * 新增协议时只需要在这一处加一行，而且这行是纯追加，不改变原有逻辑。
 */

#include <stdint.h>
#include <stdbool.h>
#include "protocol/engine.h"

#define PROTO_MAX 8u

/* 注册一个协议模块。重复的 proto_id 会被拒绝（返回 false）。 */
bool proto_register(const proto_module_t *mod);

/* 按 proto_id 查找；找不到返回 NULL */
const proto_module_t *proto_find(uint8_t proto_id);

/* 遍历用：已注册数量与第 idx 个模块（顺序即注册顺序） */
uint32_t proto_count(void);
const proto_module_t *proto_at(uint32_t idx);

/*
 * 注册所有内置协议模块。由 main 在中间件初始化之后、应用层启动之前调用。
 * 实现放在 protocol/registry_builtin.c，新增协议只改这个文件。
 */
void proto_init(void);

#endif /* PROTO_REGISTRY_H */
