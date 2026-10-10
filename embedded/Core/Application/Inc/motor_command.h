/**
  ******************************************************************************
  * @file    motor_command.h
  * @brief   电机步进指令：解析 USB 文本命令并向控制中断传递步进事件。
  ******************************************************************************
  */

#ifndef MOTOR_COMMAND_H_
#define MOTOR_COMMAND_H_

/* 头文件包含 ------------------------------------------------*/
#include <stdint.h>

// USB 中断接收 ASCII 命令行，1 ms 控制中断读取合并后的步进数。
/* 公开函数声明 ------------------------------------------------*/
/**
  * @brief  按字节接收并解析 USB 步进命令。
  * @param  data: 接收数据指针，可包含半条命令或多条命令。
  * @param  length: 本次数据长度。
  * @note   由 USB 接收回调调用；以 LF 结束一行，超长或超时残留行丢弃到下一个 LF。
  * @retval 无。
  */
void Motor_CommandFeed(const uint8_t* data, uint32_t length);
/**
  * @brief  清空步进队列与命令行解析状态。
  * @note   在临界区内复位队列索引；保留累计丢弃计数。
  * @retval 无。
  */
void Motor_CommandReset(void);
/**
  * @brief  取出队列中的有效指令并按电机合并步进数。
  * @param  steps: 三元素输出数组；正值为增加步数，负值为减少步数。
  * @note   由控制中断调用；丢弃到达时间达到有效期限的命令。
  * @retval 无。
  */
void Motor_CommandTakeSteps(int16_t steps[3]);

extern volatile uint32_t motor_command_dropped;

#endif
