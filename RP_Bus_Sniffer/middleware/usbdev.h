#ifndef MW_USBDEV_H
#define MW_USBDEV_H

/*
 * 中间件层：USB 复合设备
 *
 * 赛题 1.5 / F-02：设备要同时提供
 *   - CDC 虚拟串口：串口透传助手（上位机 P-04）
 *   - Vendor(WinUSB) 通道：参数配置、可视化调试、日志实时上传（P-02/P-05）
 *   - MSC（可选）：把 TF 卡当 U 盘挂出来
 *
 * 对上层只暴露两件事：注册成 log sink（实时上传），以及收发命令帧。
 * 描述符、端点、重连逻辑都在实现里。
 */

#include <stdint.h>
#include <stdbool.h>
#include "middleware/log.h"

typedef enum {
    USB_CH_CDC = 0,     /* 虚拟串口 */
    USB_CH_VENDOR,      /* 配置/数据通道 */
} usb_chan_t;

bool usbdev_init(void);
bool usbdev_ready(void);

/* 把实时上传注册进日志系统：log_attach_sink(usbdev_sink()) */
const log_sink_t *usbdev_sink(void);

/* 向上位机发一段数据（非阻塞，内部排队）。队列满返回 false。 */
bool usbdev_tx(usb_chan_t ch, const uint8_t *data, uint32_t len);

/* 上行数据到达回调。payload 只在回调期间有效，需要留存请自行拷贝。 */
typedef void (*usbdev_cmd_cb_t)(usb_chan_t ch, const uint8_t *payload, uint16_t len);

void usbdev_set_cmd_handler(usbdev_cmd_cb_t cb);

#endif /* MW_USBDEV_H */
