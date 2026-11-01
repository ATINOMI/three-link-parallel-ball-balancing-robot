/**
  ******************************************************************************
  * @file    can.c
  * @brief   实现 CAN 电机控制命令的发送
  *          处理电机回传数据。
  * @author  ATINOMI
  * @date    2026-07-22
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can_motor.h"
#include "butterworth.h"

/* Private defines -----------------------------------------------------------*/

/** @brief 用于同时控制底盘电机的 CAN 标准帧 ID。 */
#define CAN_CHASSIS_ALL_ID  0x200U

/** @brief M3508 电机 1-3 的 CAN 标准帧 ID。 */
#define CAN_3508_M1_ID      0x201U
#define CAN_3508_M2_ID      0x202U
#define CAN_3508_M3_ID      0x203U


/** @brief 并联机器人使用的电机数量。 */
#define CAN_MOTOR_COUNT     3U
// M3508 手册的减速比，用于将转子编码器计数换算为输出轴角度。

/* Private variables ------------------------------------------------------------------*/

CAN_TxHeaderTypeDef can1_tx_header;   // CAN1 发送报文头。
uint8_t can1_tx_data[8];              // CAN1 发送数据缓冲区。
uint32_t can1_tx_mailbox;             // CAN1 发送邮箱编号。

CAN_RxHeaderTypeDef can1_rx_header;   // CAN1 接收报文头。
uint8_t can1_rx_data[8];              // CAN1 接收数据缓冲区。

/* Private types ------------------------------------------------------------------*/

static volatile motor_measure_t motor_measure[CAN_MOTOR_COUNT];    //电机1-3回传数据结构体
/* 用于判断通信是否正常 */
static volatile uint32_t motor_last_rx_ms[CAN_MOTOR_COUNT];        // 最近一帧有效反馈的接收时间
static volatile uint8_t motor_feedback_received[CAN_MOTOR_COUNT];  // 是否曾经收到过有效反馈
// 原始角度窗口测速保留；二阶低通由定时器按独立周期更新。
#define SPEED_WINDOW_MS CAN_SPEED_WINDOW_MS

typedef struct {
  int32_t  delta_sum;  // 当前测速窗口内累计的编码器增量
  uint32_t last_ms;    // 上次更新速度的时间
  ButterworthState lowpass;
} SpeedFilterState;

static SpeedFilterState speed_filter[CAN_MOTOR_COUNT];
static ButterworthCoefficients speed_filter_coefficients;
/**
 * @brief 更新电机速度滤波器。
 *
 * @param[in] index 电机索引，取值范围为 0~2。
 * @param[in] delta 编码器增量，单位为编码器计数。
 * @param[in] now   当前时间，单位为毫秒。
 * @param[in] reset 是否重置滤波器状态。
 */
static void UpdateAngleSpeed(uint8_t  index, int32_t delta,
                              uint32_t now,   uint8_t reset)
{
  /* 当前电机的测速过程状态 */
  SpeedFilterState *state         = &speed_filter[index];

  /* 当前电机的反馈与计算结果 */
  volatile motor_measure_t *motor = &motor_measure[index];

  /* 如果需要重置滤波器状态，则清除累计增量和上次更新时间，并将速度输出置零。 */
  if (reset)
  {
    state->delta_sum = 0;
    state->last_ms = now;
    state->lowpass = (ButterworthState){0};
    motor->angle_speed_rpm = 0.0f;
    motor->filtered_speed_rpm = 0.0f;
    return;
  }

  /* 累计当前增量到测速窗口内的总增量。 */
  state->delta_sum += delta;
  uint32_t elapsed_ms = (uint32_t)(now - state->last_ms);

  /* 如果当前时间与上次更新时间的差值小于测速窗口时间，则不更新速度。 */
  if (elapsed_ms < SPEED_WINDOW_MS)
  {
    return;
  }
  /* 原始角度微分速度；不再串联旧的一阶低通。 */
  float dt = elapsed_ms * 0.001f;
  float angle_speed = state->delta_sum * (60.0f / 8192.0f) / dt;
  motor->angle_speed_rpm = angle_speed;
  state->delta_sum = 0;
  state->last_ms = now;
}

void CAN_SetSpeedFilterPeriod(uint16_t period_ms) {
  Butterworth_Design(&speed_filter_coefficients, CAN_SPEED_FILTER_CUTOFF_HZ, period_ms * 0.001f);
  for (uint8_t i = 0; i < CAN_MOTOR_COUNT; ++i)
    Butterworth_Reset(&speed_filter[i].lowpass, &speed_filter_coefficients, motor_measure[i].filtered_speed_rpm);
}

void CAN_FilterSpeeds(void) {
  for (uint8_t i = 0; i < CAN_MOTOR_COUNT; ++i)
    motor_measure[i].filtered_speed_rpm = Butterworth_Update(
        &speed_filter[i].lowpass, &speed_filter_coefficients, motor_measure[i].angle_speed_rpm);
}

/* Private function prototypes---------------------------------------------------------*/

/**
  * @brief 发送 M3508 电机电流控制命令。
  *
  * 使用标准数据帧，将目标电流写入 CAN2 发送缓冲区，
  * 并通过 HAL CAN 接口发送出去。
  *
  * @param[in] current 目标电流值。
  * @retval HAL 状态码。
  */
HAL_StatusTypeDef CAN_MotorStart(void) {
  // 每次上电先清除累计角度，首帧只建立编码器基准，不计入角度变化。
  for (uint8_t i = 0; i < CAN_MOTOR_COUNT; ++i) {
    motor_measure[i] = (motor_measure_t){0};
    motor_last_rx_ms[i] = 0U;
    motor_feedback_received[i] = 0U;
    speed_filter[i] = (SpeedFilterState){0};
  }
  CAN_SetSpeedFilterPeriod(CAN_SPEED_FILTER_PERIOD_MS);
  CAN_FilterTypeDef filter = {0};               // 配置 CAN1 滤波器，接收 1-3 号电机的反馈数据。
  filter.FilterBank = 0;                        // 使用滤波器 0。
  filter.FilterMode = CAN_FILTERMODE_IDLIST;    // 使用标识符列表模式，接收指定 ID。
  filter.FilterScale = CAN_FILTERSCALE_16BIT;   // 使用 16 位标识符，允许接收多个 ID。
  filter.FilterIdHigh = CAN_3508_M1_ID << 5;    // 设置滤波器高位 ID 为 1 号电机的标准帧 ID。
  filter.FilterIdLow  = CAN_3508_M2_ID << 5;    // 设置滤波器低位 ID 为 2 号电机的标准帧 ID。
  filter.FilterMaskIdHigh = CAN_3508_M3_ID << 5;// 设置滤波器高位掩码 ID 为 3 号电机的标准帧 ID。
  filter.FilterMaskIdLow = CAN_3508_M3_ID << 5; // 设置滤波器低位掩码 ID 为 3 号电机的标准帧 ID。
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;   // 将接收的消息放入 FIFO0。
  filter.FilterActivation = ENABLE;             // 启用滤波器。
  filter.SlaveStartFilterBank = 14;             // 设置从机滤波器起始编号为 14，允许使用更多滤波器。

  HAL_StatusTypeDef status = HAL_CAN_ConfigFilter(&hcan1, &filter);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_CAN_Start(&hcan1);
  if (status != HAL_OK)
  {
    return status;
  }
  return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint8_t CAN_MotorFeedbackIsFresh(uint8_t motor_id, uint32_t timeout_ms) {
  if (motor_id < 1U || motor_id > CAN_MOTOR_COUNT) {
    return 0U;
  }
  uint8_t index = motor_id - 1U;
  return motor_feedback_received[index] &&
         (uint32_t)(HAL_GetTick() - motor_last_rx_ms[index]) <= timeout_ms;
}
HAL_StatusTypeDef CAN_cmd_motor1(int16_t current)
{
    int16_t motor_current[CAN_MOTOR_COUNT] = {current, 0, 0};

    return CAN_cmd_three(motor_current);
}

/**
  * @brief 发送底盘 1-3 号电机的电流控制命令。
  *
  * 使用标准数据帧 0x200，将 3 个电机的目标电流依次写入 CAN2 数据帧，
  * 用于同时控制麦轮底盘的 3 个电调。
  *
  * @param[in] motor_current 3 个电机的目标电流数组。
  * @retval HAL 状态码。
  */
HAL_StatusTypeDef CAN_cmd_three(const int16_t motor_current[CAN_MOTOR_COUNT])
{
    if (motor_current == NULL)
    {
        return HAL_ERROR;
    }

    can1_tx_header.IDE   = CAN_ID_STD;         // 使用标准帧标识符。
    can1_tx_header.StdId = CAN_CHASSIS_ALL_ID; // 设置标准帧 ID。
    can1_tx_header.RTR   = CAN_RTR_DATA;       // 使用数据帧。
    can1_tx_header.DLC   = 8;                  // 设置数据长度为 8 字节。

    // 第四个电机槽位保持零电流。
    can1_tx_data[6] = 0U;
    can1_tx_data[7] = 0U;

    /* 按高字节在前的顺序存储 3 个电机的目标电流值。 */
    for (uint8_t i = 0; i < CAN_MOTOR_COUNT; i++)
    {
        can1_tx_data[2U * i] = (uint8_t)(motor_current[i] >> 8);
        can1_tx_data[2U * i + 1U] = (uint8_t)(motor_current[i]);
    }

    return HAL_CAN_AddTxMessage(
        &hcan1,
        &can1_tx_header,
        can1_tx_data,
        &can1_tx_mailbox
        );
}

/**
 * @brief 电机回传数据解析
 * @param[in] *motor 电机状态结构体指针
 *            data   CAN2 接收数据缓冲区。
 */
static void get_motor_measure(
    volatile motor_measure_t* motor,
    const uint8_t data[8])
{
    motor->last_ecd = motor->ecd;

    motor->ecd            = (int16_t)((data[0] << 8) | data[1]); //拼接ecd
    motor->speed_rpm      = (int16_t)((data[2] << 8) | data[3]); //拼接speed_rpm
    motor->given_cuurent  = (int16_t)((data[4] << 8) | data[5]); //拼接given_cuurent

    motor->temperate      = data[6];
}

/**
 * @brief 接收电机电调反馈数据
 * can1总线触发接收中断后使用回调函数
 *
 * @param[in] hcan can总线句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
  if (hcan->Instance != CAN1) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                            &can1_rx_header, can1_rx_data) != HAL_OK) {
      return;
    }
    if (can1_rx_header.IDE != CAN_ID_STD ||
        can1_rx_header.RTR != CAN_RTR_DATA ||
        can1_rx_header.DLC != 8U ||
        can1_rx_header.StdId < CAN_3508_M1_ID ||
        can1_rx_header.StdId > CAN_3508_M3_ID) {
      continue;
    }

    uint8_t index = (uint8_t)(can1_rx_header.StdId - CAN_3508_M1_ID);
    uint32_t now = HAL_GetTick();
    uint8_t reset_filter = !motor_feedback_received[index] ||
        (uint32_t)(now - motor_last_rx_ms[index]) > 100U;
    int32_t delta = 0;
    get_motor_measure(&motor_measure[index], can1_rx_data);
    if (!motor_feedback_received[index]) {
      // 首帧建立相对变化量零点；输出角度由 CAN_GetMotorAngle 加 12.77 度标定值。
      motor_measure[index].last_ecd = motor_measure[index].ecd;
      motor_measure[index].total_ecd = 0;
    } else {
      delta = (int32_t)motor_measure[index].ecd -
                      motor_measure[index].last_ecd;
      if (delta > 4096) {
        delta -= 8192;
      } else if (delta < -4096) {
        delta += 8192;
      }
      motor_measure[index].total_ecd += delta;
    }
    UpdateAngleSpeed(index, delta, now, reset_filter);
    motor_last_rx_ms[index] = now;
    motor_feedback_received[index] = 1U;
  }
}

/**
 * @brief 获取1号电机电调回传的数据
 *
 * motor1_measure由CAN接收回调函数更新。通过返回只读指针，
 * 允许main函数或PID控制器读取反馈数据，同时避免外部代码直接修改
 * CAN模块内部保存的电机状态。
 *
 * @return 指向电机1回传数据结构体的只读指针。
 */
const volatile motor_measure_t *CAN_GetMotor1Measure(void)
{
    return &motor_measure[0];
}

/**
  * @brief 获取指定编号电机的电调回传数据。
  *
  * @param[in] motor_id 电机编号，取值范围为 1~4。
  * @return 指向指定电机回传数据结构体的只读指针；编号非法时返回 NULL。
  */
const volatile motor_measure_t *CAN_GetMotorMeasure(uint8_t motor_id)
{
    if (motor_id < 1U || motor_id > CAN_MOTOR_COUNT)
    {
        return NULL;
    }

    return &motor_measure[motor_id - 1U];
}

float CAN_GetMotorAngle(uint8_t motor_id) {
  const volatile motor_measure_t *motor = CAN_GetMotorMeasure(motor_id);
  if (motor == NULL) {
    return 0.0f;
  }
  return CAN_MOTOR_INITIAL_ANGLE_DEG + motor->total_ecd *
         (360.0f / (8192.0f * CAN_MOTOR_REDUCTION_RATIO));
}

void CAN_ZeroMotorAngle(uint8_t motor_id) {
  if (motor_id < 1U || motor_id > CAN_MOTOR_COUNT) {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  motor_measure[motor_id - 1U].total_ecd = 0;
  __set_PRIMASK(primask);
}