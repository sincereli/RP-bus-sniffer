#ifndef MW_LOG_CODEC_H
#define MW_LOG_CODEC_H

/*
 * 中间件层：记录编码器契约
 *
 * 每种协议自带一个 record_codec_t，由协议模块在初始化时注册。
 * 中间件只按 proto_id 查表调用，代码里没有任何 switch (proto_id)。
 *
 * 这就是"加协议不改上层"的落点：新增协议 = 新增一个模块 + 一行注册，
 * 中间件、应用层、HAL 一行不动。
 */

#include <stdint.h>
#include <stdbool.h>
#include "record.h"

typedef struct record_codec record_codec_t;

struct record_codec {
    uint8_t     proto_id;   /* proto_id_t，必须与所属协议模块一致 */
    uint16_t    fmt_ver;    /* 负载格式版本，写进会话元数据，便于以后改格式 */

    /*
     * 会话元数据：把该协议打开时的参数（波特率、CPOL/CPHA、地址模式、
     * RTT 通道号…）编码成一段字节，写进会话文件头。
     * 解析端凭这段元数据才知道怎么解后面的负载。
     *
     * cfg 是引擎打开时用的配置结构体（协议私有类型，由协议模块自己
     * 转换）。返回写入的字节数，0 表示没有元数据或失败。
     */
    uint16_t (*hdr_build)(const void *cfg, uint8_t *buf, uint16_t cap);

    /*
     * 把一条**原生记录**编码成统一负载。
     *   rec：协议私有的记录结构体（uart_event_t / i2c_transaction_t …）
     *   buf：中间件提供的缓冲区，容量 REC_PAYLOAD_MAX
     * 返回写入字节数；0 表示这条记录不落盘（例如被过滤掉）。
     * 需要截断时置 REC_F_TRUNC 并返回实际写入长度。
     *
     * 结果缓冲区由中间件持有，有效到下一次 log_emit()。
     */
    uint16_t (*encode)(const void *rec, uint8_t *buf, uint16_t cap);

    /*
     * 给屏幕用的一行摘要，例如：
     *   "I2C 0x50 W reg=0x0F ACK"
     *   "SPI CS帧 552bit 40.00MHz"
     * 由中间件在每条记录到达时调用，最新一行保存在中间件里供 UI 取用。
     */
    void (*summarize)(const void *rec, char *line, uint16_t cap);
};

#endif /* MW_LOG_CODEC_H */
