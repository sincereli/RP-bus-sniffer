#ifndef MW_CFGFILE_H
#define MW_CFGFILE_H

/*
 * 中间件层：config.txt 解析（赛题 1.4 / F-14）
 *
 * 只负责"文本键值对 <-> 内存字典"这一件事。具体某个 key 是什么意思，
 * 由各协议的 proto_cfg_ops_t::cfg_parse 解释——中间件不认识 "baud" 或
 * "cpol"，否则加协议又得改这里。
 *
 * 文件格式（与实现约定）：
 *   # 注释
 *   bus=I2C
 *   i2c.mode=sniff
 *   i2c.addr_mode=7
 *   空行与行尾注释忽略；非法行跳过并计数，不影响其它行生效。
 */

#include <stdint.h>
#include <stdbool.h>

#define CFG_KEY_MAX 24u
#define CFG_VAL_MAX 40u
#define CFG_ENTRY_MAX 48u

/* 加载配置文件。失败时字典保持为空，调用方应回退到默认配置。 */
bool cfg_load(const char *path);

/* 读取：找到返回值指针（valid 到下次 cfg_load/cfg_set），没有返回 NULL。 */
const char *cfg_get(const char *key);

/* 写入/覆盖一个键（内存里，不落盘）。 */
bool cfg_set(const char *key, const char *val);

/* 遍历：返回条目数，第 idx 项的键与值分别写到 key、val 指向的位置。
 * 用于批量交给协议模块。 */
uint32_t cfg_count(void);
bool cfg_at(uint32_t idx, const char **key, const char **val);

/* 把内存字典写回文件（上位机在线编辑 config.txt 时用）。 */
bool cfg_save(const char *path);

/* 解析失败的行数（用于告警） */
uint32_t cfg_error_count(void);

#endif /* MW_CFGFILE_H */
