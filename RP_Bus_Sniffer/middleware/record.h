#ifndef MW_RECORD_H
#define MW_RECORD_H

/*
 * 中间件层：统一记录格式
 *
 * ============================ 分层约定 ============================
 *
 *   应用层            工作模式、菜单、屏幕、上位机会话
 *   ─────────────────────────────────────────────────────────────
 *   服务与中间件层    记录编码/分发/落盘、时间戳、USB、config、掉电保护
 *   ─────────────────────────────────────────────────────────────
 *   协议引擎层        SPI / I2C / UART / SWD-RTT 各自的采集与解析
 *   ─────────────────────────────────────────────────────────────
 *   硬件抽象层+BSP    总线线路、PIO/DMA 资源、引脚与时钟
 *   ─────────────────────────────────────────────────────────────
 *   底层驱动与芯片外设 PIO 程序、DMA、GPIO、定时器、ADC、RTC、USB/SD 控制器
 *
 * 数据只有一条主通路：协议引擎产出**各自的原生记录** → 中间件按
 * 注册的编码器把它变成这里定义的统一记录 → 扇出给 TF 卡 / USB / 屏幕。
 * 中间件不知道负载里是字节流还是事务，所以新增协议不需要改它。
 *
 * 本文件只依赖 <stdint.h>，不引任何 Pico SDK 头，便于在 PC 上做语法检查。
 */

#include <stdint.h>

/* ---------------- 协议标识 ----------------
 * 追加新协议时**只在末尾加**，不要复用已删除的值：这个数字会被写进
 * 日志文件头，复用会让旧日志被按新协议解析。
 * 0 保留给"未知/无效"。 */
typedef enum {
    PROTO_UNKNOWN = 0,
    PROTO_SPI     = 1,   /* SPI 监听 / 主机读写 */
    PROTO_I2C     = 2,   /* I2C 监听 / 主机读写 */
    PROTO_UART    = 3,   /* UART 收发 */
    PROTO_SWD_RTT = 4,   /* SWD RTT 上行日志 */
    /* 5..31 预留给后续协议 */
    PROTO_ID_MAX  = 32,
} proto_id_t;

/* ---------------- 记录标志 ---------------- */
#define REC_F_TRUNC      0x01u   /* 记录被截断（缓冲不够 / 帧未结束） */
#define REC_F_ANOMALY    0x02u   /* 时序或校验异常（毛刺、SCK 占空比畸变等） */
#define REC_F_MASTER     0x04u   /* 由主机模式产生（读写外设），不是监听到的流量 */
#define REC_F_LAST       0x08u   /* 会话的最后一条（便于接收端判断结尾） */

/* ---------------- 统一记录（内存视图） ----------------
 * 这是中间件看到的形态。payload 的内容由对应协议的编码器定义，
 * 中间件只当它是不透明字节块。
 *
 * 生命周期：payload 指向的内存由**编码器内部**的缓冲区持有，有效到
 * 下一次 log_emit() 为止。所以各个 sink 必须在 log_emit() 返回前
 * 把它写完（当前三个 sink 都是同步写，满足这个前提）。
 * 这样设计的目的是避免每帧再拷一次。
 */
typedef struct {
    uint64_t       t_us;      /* 时间戳：会话起点为 0 的微秒数，单调递增 */
    uint8_t        proto_id;  /* proto_id_t */
    uint8_t        flags;     /* REC_F_* 的按位或 */
    uint16_t       len;       /* payload 字节数 */
    const uint8_t *payload;   /* 编码器产出的负载 */
} log_record_t;

/* ---------------- 线上序列化格式 ----------------
 * TF 卡与 USB 共用同一份字节布局，PC 端只需要一套解析代码。
 * 全部小端。
 *
 *   每条记录：
 *     offset 0   t_us      u64
 *     offset 8   proto_id  u8
 *     offset 9   flags     u8
 *     offset 10  len       u16
 *     offset 12  payload   len 字节
 *   总长 12 + len，头部 12 字节保证 4 字节对齐。
 *
 *   会话文件：
 *     文件头（16 字节）：magic('RPLG') u32 | version u16 | hdr_bytes u16 |
 *                        session_flags u32 | 首条记录前的元数据长度 u32
 *     之后是"元数据区"：每条已注册协议的 hdr_build() 产出依次拼接，
 *       每块前有 4 字节 (proto_id u8 | pad u8 | len u16)。
 *     再之后是连续的记录，直到文件结束。
 *
 * 版本 1 是早期验证固件用的 RPSTREAM.BIN（8 字节记录头、无会话头）。
 * 本格式定为版本 2，PC 端按 magic + version 分流。
 */
#define REC_MAGIC        0x474C5052u   /* 'RPLG' 小端 */
#define REC_VERSION      2u
#define REC_HDR_BYTES    12u
#define REC_SESSION_HDR  16u

/* 一次编码最多能产出多少字节。超出的记录会被截断并置 REC_F_TRUNC。
 * 40MHz 下 64 字节的 SPI 读帧约 1.4KB 原始采样，留到 4KB 余量充足。 */
#define REC_PAYLOAD_MAX  4096u

#endif /* MW_RECORD_H */
