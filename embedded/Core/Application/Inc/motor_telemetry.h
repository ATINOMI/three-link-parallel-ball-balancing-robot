/**
  ******************************************************************************
  * @file    motor_telemetry.h
  * @brief   电机遥测：读取一致状态快照并通过 USB CDC 输出文本数据。
  ******************************************************************************
  */

#ifndef KEY_MOTOR_TELEMETRY_H_
#define KEY_MOTOR_TELEMETRY_H_

// 在主循环轮询，每 100 ms 尝试通过 USB CDC 发送三行电机状态。
/* 公开函数声明 ------------------------------------------------*/
/**
  * @brief  周期性发送三路电机状态文本。
  * @note   在主循环调用，每 100 ms 尝试发送；快照复制后再格式化，USB 未就绪时跳过。
  * @retval 无。
  */
void Motor_TelemetryPoll(void);

#endif  // KEY_MOTOR_TELEMETRY_H_
