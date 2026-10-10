/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN1 板级通信接口：配置接收过滤器、分发接收数据并发送标准帧。
  ******************************************************************************
  */

#ifndef KEY_BSP_CAN_H_
#define KEY_BSP_CAN_H_

/* 头文件包含 ------------------------------------------------*/
#include "main.h"

/* 类型定义 ------------------------------------------------*/
typedef void (*BSP_CAN_ReceiveCallback)(const CAN_RxHeaderTypeDef *header,
                                       const uint8_t data[8]);

// CAN1 板级通信，仅接收指定标识符的标准数据帧。
/* 公开函数声明 ------------------------------------------------*/
/**
  * @brief  初始化 CAN1 接收过滤器并启动接收中断。
  * @param  ids: 标准帧标识符数组，每项取值为 0～0x7FF。
  * @param  count: 过滤器数量，取值为 1～14。
  * @param  callback: 接收回调函数，不允许为空。
  * @note   MX_CAN1_Init 必须先执行；回调在 CAN1 接收中断中运行。
  * @retval HAL_OK 表示成功；参数或 HAL 配置失败时返回错误状态。
  */
HAL_StatusTypeDef BSP_CAN_Init(const uint16_t *ids, uint8_t count,
                             BSP_CAN_ReceiveCallback callback);
/**
  * @brief  将一帧标准数据帧提交到 CAN1 发送邮箱。
  * @param  id: 11 位标准帧标识符。
  * @param  data: 待发送数据指针，不允许为空。
  * @param  length: 数据字节数，取值为 0～8。
  * @note   不等待总线发送完成；HAL_OK 不代表接收端已收到数据。
  * @retval HAL_OK 表示已提交；邮箱满返回 HAL_BUSY；其他失败返回 HAL 错误状态。
  */
HAL_StatusTypeDef BSP_CAN_Send(uint16_t id, const uint8_t *data, uint8_t length);
/**
  * @brief  读取 CAN1 的总线关闭状态。
  * @retval 1 表示处于 Bus-Off 状态，0 表示未处于该状态。
  */
uint8_t BSP_CAN_IsBusOff(void);
/**
  * @brief  请求取消 CAN1 三个发送邮箱中尚未完成的发送。
  * @note   用于丢弃旧控制帧；不等待取消完成，也不能撤回已发送的数据。
  * @retval 无。
  */
void BSP_CAN_AbortPending(void);

#endif  // KEY_BSP_CAN_H_
