/**
  ******************************************************************************
  * @file    pid.h
  * @brief   pid控制结构体定义。
  * @author  ATINOMI
  * @date    2026-07-23
  ******************************************************************************
  */
#ifndef REMOTECONTROL_PID_H
#define REMOTECONTROL_PID_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID 控制器结构体
 *
 * 用于速度环闭环控制
 */
typedef struct
{
    float kp;           ///< 比例系数
    float ki;           ///< 积分系数
    float kd;           ///< 微分系数

    float max_output;   ///< 输出最大限幅
    float max_ki_output;///< 积分项最大限幅

    float setpoint;     ///< 设定值
    float feedback;     ///< 反馈值
    float error;        ///< 误差值

    float p_output;     ///< 比例输出
    float i_output;     ///< 积分输出
    float d_output;     ///< 微分输出
    float output;       ///< 总输出

    float previous_error;
    float dt;           ///< 控制周期

    bool initialized;
}Pid_type;

/* Exported functions --------------------------------------------------------*/

void Pid_Init(Pid_type *pid,
              float kp,
              float ki,
              float kd,
              float max_output,
              float max_ki_output,
              float dt);

float Pid_update(Pid_type *pid, float setpoint, float feedback);

void Pid_Reset(Pid_type *pid);

#endif //REMOTECONTROL_PID_H
