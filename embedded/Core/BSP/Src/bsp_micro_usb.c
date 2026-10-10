/**
  ******************************************************************************
  * @file    bsp_micro_usb.c
  * @brief   Micro USB CDC 板级通信接口：管理收发回调和非阻塞发送缓冲区。
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------*/
#include "bsp_micro_usb.h"

#include <string.h>

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* 私有定义与变量 ------------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS;
static BSP_MicroUsbReceiveCallback receive_callback;
static BSP_MicroUsbResetCallback reset_callback;
/* USB 异步发送持有该内存，仅在上一笔传输完成后改写。 */
static uint8_t tx_buffer[768];

/* 函数实现 ------------------------------------------------*/
/**
  * @brief  注册 USB CDC 的接收和复位回调。
  * @param  receive: 接收数据回调，可为空。
  * @param  reset: CDC 初始化或反初始化时调用的复位回调，可为空。
  * @note   在启动 USB 设备之前调用；本函数只注册回调，不初始化 USB 外设。
  * @retval 无。
  */
void BSP_MicroUsbInit(BSP_MicroUsbReceiveCallback receive,
                      BSP_MicroUsbResetCallback reset) {
  receive_callback = receive;
  reset_callback = reset;
}

/**
  * @brief  检查 USB 是否已完成枚举且发送通道空闲。
  * @note   调用方需屏蔽中断，避免检查过程中 USB 断开导致类数据失效。
  * @retval 1 表示可以发送，0 表示设备未就绪或发送忙。
  */
static uint8_t UsbReady(void) {
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED ||
      hUsbDeviceFS.pClassData == NULL) return 0U;
  USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
  return cdc->TxState == 0U;
}

/**
  * @brief  在临界区内查询 USB CDC 发送就绪状态。
  * @retval 1 表示就绪，0 表示未就绪；结果只反映查询时刻的状态。
  */
uint8_t BSP_MicroUsbReady(void) {
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  uint8_t ready = UsbReady();
  __set_PRIMASK(saved);
  return ready;
}

/**
  * @brief  复制数据并尝试启动一次非阻塞 USB CDC 发送。
  * @param  data: 待发送数据指针。
  * @param  length: 数据长度，取值为 1～768 字节。
  * @note   BSP 持有传输缓冲区，调用返回后调用方可以复用原始数据缓冲区。
  * @retval 1 表示已启动发送，0 表示参数无效、USB 未就绪或提交失败。
  */
uint8_t BSP_MicroUsbSend(const uint8_t *data, uint16_t length) {
  if (data == NULL || length == 0U || length > sizeof(tx_buffer)) return 0U;
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  uint8_t sent = 0U;
  /* 检查、复制和提交在同一临界区，避免 USB 状态中途改变。 */
  if (UsbReady()) {
    memcpy(tx_buffer, data, length);
    sent = CDC_Transmit_FS(tx_buffer, length) == USBD_OK;
  }
  __set_PRIMASK(saved);
  return sent;
}

/**
  * @brief  将 CDC 收到的数据交给应用层。
  * @param  data: 本次接收数据，应用回调需在返回前完成读取。
  * @param  length: 本次接收字节数。
  * @note   由 CDC 适配层在 USB 中断中调用；数据包边界不等于命令行边界。
  * @retval 无。
  */
void BSP_MicroUsbOnReceive(const uint8_t *data, uint32_t length) {
  if (receive_callback != NULL) receive_callback(data, length);
}

/**
  * @brief  通知应用层清除 USB 命令接收状态。
  * @note   由 CDC 初始化或反初始化路径调用，避免连接切换后继续执行旧命令。
  * @retval 无。
  */
void BSP_MicroUsbOnReset(void) {
  if (reset_callback != NULL) reset_callback();
}
