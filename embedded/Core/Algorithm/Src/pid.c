/**
  ******************************************************************************
  * @file    pid.c
  * @brief   PID 控制器的初始化、计算与复位功能实现。
  *          当前控制器采用位置式 PID 结构，支持比例、积分、微分项，
  *          并提供积分项限幅、输出限幅和积分抗饱和处理。
  * @author  ATINOMI
  * @date    2026-07-23
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------------------------*/
#include "pid.h"

/* 私有函数实现 ----------------------------------------------*/

/**
  * @brief 将浮点数限制在指定范围内。
  *
  * 当输入值大于上限时返回上限；当输入值小于下限时返回下限；
  * 输入值位于范围内时保持不变。
  *
  * @param[in] value      待限制的输入值。
  * @param[in] min_value  允许的最小值。
  * @param[in] max_value  允许的最大值。
  * @retval 限制范围内的浮点数。
  */
static float Pid_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/* 公开函数实现 --------------------------------------------------------*/

/**
  * @brief 初始化 PID 控制器。
  *
  * 设置 PID 控制器的比例、积分、微分参数，配置输出限幅和控制周期，
  * 最后清除控制器的历史运行状态。
  *
  * @param[in,out] pid             PID 控制器结构体指针。
  * @param[in]     kp              比例系数。
  * @param[in]     ki              积分系数，单位时间内的积分增益。
  * @param[in]     kd              微分系数。
  * @param[in]     max_output      PID 最终输出的绝对值最大限幅。
  * @param[in]     max_ki_output   积分项输出的绝对值最大限幅。
  * @param[in]     dt              PID 控制周期，单位：秒。
  * @retval None
  */

void Pid_Init(Pid_type *pid,
              float kp,
              float ki,
              float kd,
              float max_output,
              float max_ki_output,
              float dt)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->max_output = max_output;
    pid->max_ki_output = max_ki_output;
    pid->dt = dt;

    /* 初始化后清除积分、微分和历史反馈等运行状态。 */
    Pid_Reset(pid);
}

/**
  * @brief 计算一次 PID 控制输出。
  *
  * 本函数使用位置式 PID 算法：
  *
  *     output = P + I + D
  *
  * 积分项按照控制周期累加，并限制积分项的最大值，防止长时间误差
  * 导致积分饱和。微分项作用于反馈值变化量，可以减少目标值突变时
  * 产生的微分冲击。
  *
  * 当 PID 输出已经达到上限或下限，且当前误差会使输出继续向饱和方向
  * 变化时，暂时停止积分累加，从而实现积分抗饱和。
  *
  * @param[in,out] pid       PID 控制器结构体指针。
  * @param[in]     setpoint  目标值，本工程为输出轴目标角度（度）。
  * @param[in]     feedback  反馈值，本工程为编码器换算的输出轴角度（度）。
  * @return PID 控制输出，本工程为 C620 目标电流协议原始值。
  */
float Pid_update(Pid_type *pid, float setpoint, float feedback)
{
    if (pid == NULL || pid->dt <= 0.0f)
    {
        return 0.0f;
    }

    pid->setpoint = setpoint;
    pid->feedback = feedback;
    pid->error    = setpoint - feedback;

    /* 计算比例项：当前误差越大，比例输出越大。 */
    pid->p_output = pid->kp * pid->error;

    /*
     * 计算积分项候选值。
     * dt 必须使用秒为单位，例如 10 ms 对应 0.01 s。
     */
    float new_i_output =
        pid->i_output + pid->ki * pid->error * pid->dt;

    /* 限制积分项，避免积分项无限累积。 */
    new_i_output = Pid_Clamp(new_i_output,
                             -pid->max_ki_output,
                             pid->max_ki_output);

    /*
     * 微分项作用于反馈值，而不是误差值：
     *
     *     D = -Kd * (feedback(k) - feedback(k-1)) / dt
     *
     * 第一次计算时没有有效的历史反馈，因此将微分项置零。
     */
    if (pid->initialized)
    {
        pid->d_output = -pid->kd *
                        (feedback - pid->previous_feedback) /
                        pid->dt;
    }
    else
    {
        pid->d_output = 0.0f;
    }

    /* 使用新的积分项预估本次未限幅输出。 */
    float candidate_output =
        pid->p_output + new_i_output + pid->d_output;

    /* 判断预估输出是否超过上限或下限。 */
    bool output_saturated_high =
        candidate_output > pid->max_output;

    bool output_saturated_low =
        candidate_output < -pid->max_output;

    /* 判断积分项是否会使当前的输出饱和更加严重。 */
    bool integral_increases_saturation =
        (output_saturated_high && pid->error > 0.0f) ||
        (output_saturated_low && pid->error < 0.0f);

    /* 只有积分项不会加重饱和时，才更新积分状态。 */
    if (!integral_increases_saturation)
    {
        pid->i_output = new_i_output;
    }

    /* 对最终 PID 输出进行限幅。 */
    pid->output = Pid_Clamp(
        pid->p_output + pid->i_output + pid->d_output,
        -pid->max_output,
        pid->max_output);

    /* 保存本次反馈值，供下一次计算微分项使用。 */
    pid->previous_feedback = feedback;
    pid->initialized = true;

    return pid->output;
}

/**
  * @brief 清除 PID 控制器的历史状态。
  *
  * 复位后会清除积分项、微分项、输出值和上一周期反馈值，
  * 但不会修改 PID 参数和输出限幅配置。
  *
  * @param[in,out] pid PID 控制器结构体指针。
  * @retval None
  */
void Pid_Reset(Pid_type *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;

    pid->p_output = 0.0f;
    pid->i_output = 0.0f;
    pid->d_output = 0.0f;
    pid->output = 0.0f;

    pid->previous_feedback = 0.0f;
    pid->initialized = false;
}
