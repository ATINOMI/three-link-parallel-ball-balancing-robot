/**
  ******************************************************************************
  * @file    motor_control.h
  * @brief   三路电机角度控制：解析电调反馈、累计输出轴角度并计算目标电流。
  ******************************************************************************
  */

#ifndef KEY_MOTOR_CONTROL_H_
#define KEY_MOTOR_CONTROL_H_

/* 头文件包含 ------------------------------------------------*/
#include "main.h"

/* 控制参数 ------------------------------------------------*/
// 控制 M3508/C620 电调 1～3，角度统一按减速箱输出轴计算。
#define MOTOR_CONTROL_COUNT 3U  /* 受控电机数量，当前协议固定对应电调 1～3。 */
#define MOTOR_GEAR_RATIO (3591.0f / 187.0f)  /* 转子转数与输出轴转数之比。 */
// 将编码器和电流的原始方向映射为用户约定的输出轴正方向。
// 反馈角度与 PID 输出同时乘以方向系数，保证负反馈方向一致。
#define MOTOR_DIRECTION (-1.0f)
#define MOTOR_KEY_STEP_DEG 10.0f  /* 单次步进的输出轴角度，单位：度。 */
#define MOTOR_FEEDBACK_TIMEOUT_MS 100U  /* 有效反馈超时阈值，单位：毫秒。 */
// 角度单位为输出轴度数，电流单位为协议原始值；增益需按实际负载调整。
#define MOTOR_CONTROL_PERIOD_S 0.001f  /* PID 控制周期，单位：秒。 */
#define MOTOR_ANGLE_KP 100.0f  // 比例增益，单位：原始电流值/度
#define MOTOR_ANGLE_KI 10.0f   // 积分增益，单位：原始电流值/(度·秒)
#define MOTOR_ANGLE_KD 5.0f     // 微分增益，单位：原始电流值·秒/度，对反馈求导
#define MOTOR_MAX_CURRENT 3000.0f  /* 电流指令绝对值上限，单位为协议原始值，并非安培。 */
#define MOTOR_MAX_INTEGRAL_CURRENT 1000.0f  /* 积分项电流绝对值上限。 */

/* 类型定义 ------------------------------------------------*/
typedef struct {
  uint16_t ecd;  /*!< 转子单圈编码器值，范围 0～8191。 */
  int16_t speed_rpm;  /*!< 转子转速，单位 rpm，保留电调协议方向。 */
  int16_t given_current;  /*!< 反馈电流原始值，保留电调协议方向。 */
  uint8_t temperature;  /*!< 电调反馈温度，单位：摄氏度。 */
  int64_t relative_counts;  /*!< 从相对零点开始累计的转子编码器计数。 */
  uint32_t last_rx_ms;  /*!< 最近一次有效反馈的系统时间，单位：毫秒。 */
  uint8_t valid;  /*!< 是否已收到有效反馈；不等同于当前在线状态。 */
} motor_measure_t;

/* 公开状态变量 ------------------------------------------------*/
// 可在调试器中观察以下数组；目标角度支持多圈累计，不按 360°取模。
extern volatile float motor_target_angle_deg[MOTOR_CONTROL_COUNT];
extern volatile float motor_actual_angle_deg[MOTOR_CONTROL_COUNT];
extern volatile int16_t motor_current_command[MOTOR_CONTROL_COUNT];
extern volatile uint8_t motor_online[MOTOR_CONTROL_COUNT];
extern volatile uint32_t motor_tx_errors;

/* 公开函数声明 ------------------------------------------------*/
/**
  * @brief  初始化三路角度控制器并注册 CAN、USB 通信回调。
  * @note   先完成 CAN 外设初始化，再调用本函数，最后启动 USB 和 TIM6。
  * @retval HAL_OK 表示成功，其他值表示 CAN 初始化失败。
  */
HAL_StatusTypeDef Motor_Init(void);
// 电调编号 1～3 分别对应调试数组下标 0～2。
/**
  * @brief  读取指定电机的一致反馈快照。
  * @param  motor_id: 电调编号，取值为 1～3，对应数组下标 0～2。
  * @param  measure: 用于接收反馈快照的结构体指针。
  * @note   复制期间屏蔽中断，避免 64 位计数撕裂；成功不表示反馈在线。
  * @retval HAL_OK 表示复制成功，参数无效时返回 HAL_ERROR。
  */
HAL_StatusTypeDef Motor_GetMeasure(uint8_t motor_id,
                                     motor_measure_t *measure);
// 每 1 ms 的 TIM6 中断调用一次，pressed 表示消抖后的按下事件。
/**
  * @brief  执行一次三路电机角度闭环并发送合并电流帧。
  * @param  pressed: 消抖后的按下事件；非零时为全部在线电机增加一个步进。
  * @note   仅由 TIM6 每 1 ms 调用一次；每路独立检测反馈超时和重建目标。
  * @retval 无。
  */
void Motor_ControlTick(uint8_t pressed);

#endif  // KEY_MOTOR_CONTROL_H_
