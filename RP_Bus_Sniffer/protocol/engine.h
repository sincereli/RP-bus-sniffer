#ifndef PROTO_ENGINE_H
#define PROTO_ENGINE_H

/*
 * 协议引擎层：引擎契约
 *
 * 一个协议模块（proto_module_t）把四样东西打包注册：
 *   - 监听引擎   engine_ops_t    从机监听抓包（SPI/I2C/UART/RTT 都有）
 *   - 主机器     bus_master_ops_t 主动读写外设（只有 I2C/SPI 有，可留 NULL）
 *   - 编码器     record_codec_t   交给中间件统一处理
 *   - 参数       proto_cfg_ops_t  config.txt 与菜单参数
 *
 * 加一个新协议 = 新增一个目录实现这四样 + 在 proto_init() 里加一行注册。
 * 本文件与上层、下层都不互相包含具体实现。
 */

#include <stdint.h>
#include <stdbool.h>
#include "middleware/record.h"
#include "middleware/log_codec.h"

/* ---------------- 监听引擎（从机模式） ---------------- */
typedef struct {
    /*
     * 按协议私有配置打开引擎：配置总线复用、申请 PIO/DMA、启动采样。
     * cfg 指向协议自己的配置结构体（类型由模块内部约定）。
     * 失败时返回 false，错误原因由模块通过自己的状态接口提供。
     */
    bool (*open)(const void *cfg);

    /* 停止采样、释放 PIO/DMA、把总线置回高阻 */
    void (*close)(void);

    /*
     * 取一条**已解析的原生记录**，非阻塞。
     * 返回 NULL 表示当前没有新记录 —— 主循环不会因此被拖住。
     * 返回的指针有效到下一次 poll()，中间件必须在此之前用完。
     */
    const void *(*poll)(void);
} engine_ops_t;

/* ---------------- 主机模式（主动读写外设） ----------------
 * 赛题 2.2 的离线寄存器读写、上位机可视化调试都走这里。
 * 事务描述用统一的 bus_xfer_t，协议模块自己翻译成 I2C/SPI 时序。
 */
typedef struct {
    uint8_t  addr;      /* I2C 从机地址；SPI 用作片选索引；UART 忽略 */
    uint8_t  reg;       /* 寄存器地址 */
    uint8_t  reg_len;   /* 写寄存器地址的字节数，0 = 不发寄存器地址 */
    bool     read;      /* true = 读，false = 写 */
    uint8_t *data;      /* 读：接收缓冲；写：待发数据 */
    uint16_t len;       /* data 的有效长度 */
} bus_xfer_t;

typedef struct {
    bool (*open)(const void *cfg);     /* 把总线切成主机模式 */
    void (*close)(void);               /* 交还总线 */
    bool (*xfer)(bus_xfer_t *x);       /* 一次事务，成功时 x->data/len 已更新 */
} bus_master_ops_t;

/* ---------------- 协议参数 ----------------
 * config.txt（P0 F-14）与屏幕菜单共用同一套参数描述。
 * key/value 是纯文本，具体含义由协议模块解释，中间件不解析。
 */
typedef struct {
    /* 把一组 key=value 应用到一个协议私有配置结构体上。
     * 返回 false 表示有非法值，调用方应回退到默认配置。 */
    bool (*cfg_parse)(const char *const *keys, const char *const *vals,
                      uint32_t n, void *cfg_out);

    /* 把配置格式化成 config.txt 的文本行（每行 "key=value\n"）。
     * 返回写入字节数。 */
    uint32_t (*cfg_format)(const void *cfg, char *buf, uint32_t cap);

    /* 协议配置结构体的大小，供上层分配存放空间 */
    uint16_t cfg_size;

    /* 填默认配置，返回 false 表示该协议没有可用默认值 */
    bool (*cfg_default)(void *cfg_out);
} proto_cfg_ops_t;

/* ---------------- 协议模块（注册单元） ---------------- */
typedef struct {
    uint8_t  proto_id;                  /* proto_id_t，必须与编码器一致 */
    const char *name;                   /* 菜单/日志里显示的名字，如 "I2C" */

    const engine_ops_t     *sniff;      /* 可为 NULL（例如纯主机协议） */
    const bus_master_ops_t *master;     /* 可为 NULL */
    const record_codec_t   *codec;      /* 必需 */
    const proto_cfg_ops_t  *cfg;        /* 可为 NULL（无参数协议） */
} proto_module_t;

#endif /* PROTO_ENGINE_H */
