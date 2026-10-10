/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN1 板级通信接口：配置接收过滤器、分发接收数据并发送标准帧。
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------*/
#include "bsp_can.h"

/* 私有定义与变量 ------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
static BSP_CAN_ReceiveCallback receive_callback;

/* 函数实现 ------------------------------------------------*/
/**
  * @brief  初始化 CAN1 接收过滤器并启动接收中断。
  * @param  ids: 标准帧标识符数组，每项取值为 0～0x7FF。
  * @param  count: 过滤器数量，取值为 1～14。
  * @param  callback: 接收回调函数，不允许为空。
  * @note   MX_CAN1_Init 必须先执行；回调在 CAN1 接收中断中运行。
  * @retval HAL_OK 表示成功；参数或 HAL 配置失败时返回错误状态。
  */
HAL_StatusTypeDef BSP_CAN_Init(const uint16_t *ids, uint8_t count,
                             BSP_CAN_ReceiveCallback callback) {
  if (ids == NULL || callback == NULL || count == 0U || count > 14U) {
    return HAL_ERROR;
  }
  for (uint8_t i = 0; i < count; ++i) {
    if (ids[i] > 0x7FFU) return HAL_ERROR;
  }
  /* 32 位掩码过滤器同时匹配 11 位 ID、帧类型和数据帧标志。 */
  CAN_FilterTypeDef filter = {0};
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterMaskIdHigh = 0x7FFU << 5;
  filter.FilterMaskIdLow = 0x6U;  // 匹配 IDE=0 和 RTR=0，仅接收标准数据帧。
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  /* CAN1 使用过滤器组 0～13；组 14 起预留给 CAN2。 */
  filter.SlaveStartFilterBank = 14;
  for (uint8_t i = 0; i < count; ++i) {
    filter.FilterBank = i;
    filter.FilterIdHigh = ids[i] << 5;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return HAL_ERROR;
  }
  receive_callback = callback;
  if (HAL_CAN_Start(&hcan1) != HAL_OK) return HAL_ERROR;
  return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  将一帧标准数据帧提交到 CAN1 发送邮箱。
  * @param  id: 11 位标准帧标识符。
  * @param  data: 待发送数据指针，不允许为空。
  * @param  length: 数据字节数，取值为 0～8。
  * @note   不等待总线发送完成；HAL_OK 不代表接收端已收到数据。
  * @retval HAL_OK 表示已提交；邮箱满返回 HAL_BUSY；其他失败返回 HAL 错误状态。
  */
HAL_StatusTypeDef BSP_CAN_Send(uint16_t id, const uint8_t *data, uint8_t length) {
  if (data == NULL || id > 0x7FFU || length > 8U) return HAL_ERROR;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) return HAL_BUSY;
  CAN_TxHeaderTypeDef header = {0};
  uint8_t payload[8] = {0};
  uint32_t mailbox;
  header.StdId = id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = length;
  for (uint8_t i = 0; i < length; ++i) payload[i] = data[i];
  return HAL_CAN_AddTxMessage(&hcan1, &header, payload, &mailbox);
}

/**
  * @brief  读取 CAN1 的总线关闭状态。
  * @retval 1 表示处于 Bus-Off 状态，0 表示未处于该状态。
  */
uint8_t BSP_CAN_IsBusOff(void) {
  return (hcan1.Instance->ESR & CAN_ESR_BOFF) != 0U;
}

/**
  * @brief  请求取消 CAN1 三个发送邮箱中尚未完成的发送。
  * @note   用于丢弃旧控制帧；不等待取消完成，也不能撤回已发送的数据。
  * @retval 无。
  */
void BSP_CAN_AbortPending(void) {
  HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 |
                                  CAN_TX_MAILBOX2);
}

/**
  * @brief  处理 CAN1 接收 FIFO0 中待取出的报文。
  * @param  hcan: 触发回调的 CAN 外设句柄。
  * @note   由 HAL 在中断中调用；逐帧取出数据，协议解析交给注册的应用回调。
  * @retval 无。
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  if (hcan->Instance != CAN1) return;
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0U) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
    if (receive_callback != NULL) receive_callback(&header, data);
  }
}
