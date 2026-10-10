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

/* 头文件包含 ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>

/* 公开类型定义 ------------------------------------------------------------*/

/**
 * @brief PID 控制器结构体
 *
 * 通用位置式 PID；本工程用于输出轴角度闭环，输出目标电流
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

    float previous_feedback;  ///< 上一周期反馈值，用于反馈微分。
    float dt;           ///< 控制周期

    bool initialized;   ///< 是否已有历史反馈，首次计算时关闭微分项。
}Pid_type;

/* 公开函数实现 --------------------------------------------------------*/

/**
  * @brief  设置 PID 参数和限幅，并清除历史运行状态。
  * @param  pid: 控制器结构体指针。
  * @param  kp: 比例增益。
  * @param  ki: 单位时间积分增益。
  * @param  kd: 对反馈变化率求导的微分增益。
  * @param  max_output: 最终输出的绝对值限幅，应为非负数。
  * @param  max_ki_output: 积分项的绝对值限幅，应为非负数。
  * @param  dt: 控制周期，单位为秒，必须大于零。
  * @retval 无。
  */
void Pid_Init(Pid_type *pid,
              float kp,
              float ki,
              float kd,
              float max_output,
              float max_ki_output,
              float dt);

/**
  * @brief  计算一次带输出限幅、积分抗饱和和反馈微分的 PID 输出。
  * @param  pid: 控制器结构体指针。
  * @param  setpoint: 目标值，本工程为输出轴目标角度，单位为度。
  * @param  feedback: 实际值，本工程为输出轴相对角度，单位为度。
  * @retval 限幅后的控制输出；指针为空或周期无效时返回零。
  */
float Pid_update(Pid_type *pid, float setpoint, float feedback);

/**
  * @brief  清除积分、微分、输出及反馈历史，保留参数和限幅配置。
  * @param  pid: 控制器结构体指针。
  * @retval 无。
  */
void Pid_Reset(Pid_type *pid);

#endif //REMOTECONTROL_PID_H
