/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN 电机控制，处理电机回传数据
  *          接口声明。
  * @author  ATINOMI
  * @date    2026-07-22
  ******************************************************************************
  */

#ifndef REMOTECONTROL_CAN_H
#define REMOTECONTROL_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"

// 上电/重新建立角度基准时，当前输出轴角度标定为 12.77 度。
#define CAN_MOTOR_INITIAL_ANGLE_DEG 12.77f
#define CAN_MOTOR_REDUCTION_RATIO (3591.0f / 187.0f)
#define CAN_SPEED_WINDOW_MS 5U
#define CAN_SPEED_FILTER_PERIOD_MS 5U
#define CAN_SPEED_FILTER_CUTOFF_HZ 20.0f

/* External variables --------------------------------------------------------*/

/** @brief CAN1 外设句柄，由 HAL 初始化。 */
extern CAN_HandleTypeDef hcan1;

/**
  * @brief 发送 M3508 电机电流控制命令。
  *
  * @param[in] current 目标电流值。
  * @retval HAL 状态码。
  */

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 电机状态结构体
 *        ecd          由DATA[0]与DATA[1]高低8位合成
 *        speed_rpm    由DATA[2]与DATA[3]高低8位合成
 *        given_cuurent由DATA[4]与DATA[5]高低8位合成
 *        last_ecd     由前次ecd传入
 */
typedef struct
{
    int32_t total_ecd;          // 相对首次反馈/设零位置的累计编码器计数。
    uint16_t ecd;               //转子机械角度
    float angle_speed_rpm;      // 角度窗口测速，转子 rpm。
    float filtered_speed_rpm;   // 角度测速经二阶Butterworth后的转子rpm，供PID使用。
    int16_t speed_rpm;          //电调原始转子转速，仅供观察，不参与闭环
    int16_t given_cuurent;      //实际转矩电流
    uint8_t temperate;          //电机温度
    int16_t last_ecd;           //上次转子机械角度
}motor_measure_t;

/* Private function prototypes---------------------------------------------------------*/

/**
  * @brief 发送 M3508 电机电流控制命令。
  *
  * 使用标准数据帧，将目标电流写入 CAN1 发送缓冲区，
  * 并通过 HAL CAN 接口发送出去。
  *
  * @param[in] current 目标电流值。
  * @retval HAL 状态码。
  */
/** @brief 配置反馈滤波器，启动 CAN1 并开启 FIFO0 接收通知。 */
HAL_StatusTypeDef CAN_MotorStart(void);
// 在1ms定时中断分频后调用；配置函数只在启动/上位机调频时调用。
void CAN_FilterSpeeds(void);
void CAN_SetSpeedFilterPeriod(uint16_t period_ms);

/** @brief 已收到反馈且最后一帧距今不超过 timeout_ms 时返回 1。 */
uint8_t CAN_MotorFeedbackIsFresh(uint8_t motor_id, uint32_t timeout_ms);
HAL_StatusTypeDef CAN_cmd_motor1(int16_t current);

/**
  * @brief 发送 1-3 号电机的电流控制命令。
  *
  * 使用标准数据帧 0x200，将 3 个电机的目标电流依次写入 CAN1 数据帧：
  * motor_current[0] 对应 1 号电调，motor_current[1] 对应 2 号电调，
  * motor_current[2] 对应 3 号电调。
  *
  * @param[in] motor_current 3 个电机的目标电流数组。
  * @retval HAL 状态码。
  */
HAL_StatusTypeDef CAN_cmd_three(const int16_t motor_current[3]);

/**
 * @brief 接收电机电调反馈数据
 * can1总线触发接收中断后使用回调函数
 *
 * @param[in] hcan can总线句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);

/**
 *
 * @brief 获取1号电机电调回传的数据
 *
 * 该接口返回电机1回传数据结构体的地址，主函数或PID控制器可以通过
 * 该接口读取转子机械角度、转速、实际转矩电流和电机温度等信息。
 * 返回的数据由CAN接收中断回调函数更新，主函数不需要手动调用接收回调。
 *
 * @return 指向电机1回传数据结构体的只读指针。
 */

const volatile motor_measure_t *CAN_GetMotor1Measure(void);

/**
  * @brief 获取指定编号电机的电调回传数据。
  *
  * @param[in] motor_id 电机编号，取值范围为 1~3。
  * @return 指向指定电机回传数据结构体的只读指针；编号非法时返回 NULL。
  */
const volatile motor_measure_t *CAN_GetMotorMeasure(uint8_t motor_id);

// 输出轴标定角度 = 12.77 度 + 相对首次反馈的变化量；减速比 3591/187。
// 断电或丢失圈数后需重新建立基准，上升时角度减小。
float CAN_GetMotorAngle(uint8_t motor_id);

// 在机构静止、反馈有效时将当前位置重新标定为 12.77 度。
void CAN_ZeroMotorAngle(uint8_t motor_id);
#endif  // REMOTECONTROL_CAN_H
