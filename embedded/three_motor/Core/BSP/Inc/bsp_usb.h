#ifndef BSP_USB_H_
#define BSP_USB_H_

#include <stdint.h>

// USB 初始化沿用 main.c 中的 MX_USB_DEVICE_Init()。
// 非阻塞发送，返回 USBD_OK / USBD_BUSY / USBD_FAIL。
// 仅在主循环调用；发送完成前，data 指向的缓冲区必须保持有效且内容不变。
// 已枚举且没有未完成的发送时返回 1。
uint8_t BSP_USB_CanSend(void);

uint8_t BSP_USB_Send(const uint8_t *data, uint16_t length);

// 在 main.c 中实现同名函数即可处理接收数据。
// 此回调在 USB 中断中执行，不要阻塞；data 仅在本次回调期间有效。
// 每次回调是一段接收数据，不保证对应一条完整的应用消息。
void BSP_USB_ReceiveCallback(const uint8_t *data, uint32_t length);

#endif  // BSP_USB_H_
