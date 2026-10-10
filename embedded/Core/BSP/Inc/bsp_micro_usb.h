/**
  ******************************************************************************
  * @file    bsp_micro_usb.h
  * @brief   Micro USB CDC 板级通信接口：管理收发回调和非阻塞发送缓冲区。
  ******************************************************************************
  */

#ifndef KEY_BSP_MICRO_USB_H_
#define KEY_BSP_MICRO_USB_H_

/* 头文件包含 ------------------------------------------------*/
#include <stdint.h>

/* 类型定义 ------------------------------------------------*/
typedef void (*BSP_MicroUsbReceiveCallback)(const uint8_t *data, uint32_t length);
typedef void (*BSP_MicroUsbResetCallback)(void);

// 启动 USB 设备前注册应用回调。
/* 公开函数声明 ------------------------------------------------*/
/**
  * @brief  注册 USB CDC 的接收和复位回调。
  * @param  receive: 接收数据回调，可为空。
  * @param  reset: CDC 初始化或反初始化时调用的复位回调，可为空。
  * @note   在启动 USB 设备之前调用；本函数只注册回调，不初始化 USB 外设。
  * @retval 无。
  */
void BSP_MicroUsbInit(BSP_MicroUsbReceiveCallback receive,
                      BSP_MicroUsbResetCallback reset);
/**
  * @brief  在临界区内查询 USB CDC 发送就绪状态。
  * @retval 1 表示就绪，0 表示未就绪；结果只反映查询时刻的状态。
  */
uint8_t BSP_MicroUsbReady(void);
// 非阻塞发送：复制到 BSP 缓冲区，直至 USB 传输完成前均保持有效。
/**
  * @brief  复制数据并尝试启动一次非阻塞 USB CDC 发送。
  * @param  data: 待发送数据指针。
  * @param  length: 数据长度，取值为 1～768 字节。
  * @note   BSP 持有传输缓冲区，调用返回后调用方可以复用原始数据缓冲区。
  * @retval 1 表示已启动发送，0 表示参数无效、USB 未就绪或提交失败。
  */
uint8_t BSP_MicroUsbSend(const uint8_t *data, uint16_t length);
// 由 CubeMX 生成的 CDC 适配层在 USB 中断中调用。
/**
  * @brief  将 CDC 收到的数据交给应用层。
  * @param  data: 本次接收数据，应用回调需在返回前完成读取。
  * @param  length: 本次接收字节数。
  * @note   由 CDC 适配层在 USB 中断中调用；数据包边界不等于命令行边界。
  * @retval 无。
  */
void BSP_MicroUsbOnReceive(const uint8_t *data, uint32_t length);
/**
  * @brief  通知应用层清除 USB 命令接收状态。
  * @note   由 CDC 初始化或反初始化路径调用，避免连接切换后继续执行旧命令。
  * @retval 无。
  */
void BSP_MicroUsbOnReset(void);

#endif  // KEY_BSP_MICRO_USB_H_
