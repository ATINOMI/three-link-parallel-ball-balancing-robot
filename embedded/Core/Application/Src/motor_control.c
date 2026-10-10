/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   三路电机角度控制：解析电调反馈、累计输出轴角度并计算目标电流。
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------*/
#include "motor_control.h"

#include "bsp_can.h"
#include "bsp_micro_usb.h"
#include "motor_command.h"
#include "pid.h"

/* 私有定义与变量 ------------------------------------------------*/
static volatile motor_measure_t motor_measure[MOTOR_CONTROL_COUNT];
static Pid_type angle_pid[MOTOR_CONTROL_COUNT];
static uint8_t position_initialized[MOTOR_CONTROL_COUNT];  /* 各路相对位置参考是否已建立。 */

volatile float motor_target_angle_deg[MOTOR_CONTROL_COUNT];
volatile float motor_actual_angle_deg[MOTOR_CONTROL_COUNT];
volatile int16_t motor_current_command[MOTOR_CONTROL_COUNT];
volatile uint8_t motor_online[MOTOR_CONTROL_COUNT];
volatile uint32_t motor_tx_errors;

/* 私有函数原型 ------------------------------------------------*/
static void Motor_ReceiveFeedback(const CAN_RxHeaderTypeDef *header,
                                  const uint8_t data[8]);

/* 函数实现 ------------------------------------------------*/
/**
  * @brief  初始化三路角度控制器并注册 CAN、USB 通信回调。
  * @note   先完成 CAN 外设初始化，再调用本函数，最后启动 USB 和 TIM6。
  * @retval HAL_OK 表示成功，其他值表示 CAN 初始化失败。
  */
HAL_StatusTypeDef Motor_Init(void) {
  const uint16_t feedback_ids[] = {0x201U, 0x202U, 0x203U};
  for (uint8_t i = 0; i < MOTOR_CONTROL_COUNT; ++i) {
    Pid_Init(&angle_pid[i], MOTOR_ANGLE_KP, MOTOR_ANGLE_KI, 0.0f,
             MOTOR_MAX_CURRENT, MOTOR_MAX_INTEGRAL_CURRENT,
             MOTOR_CONTROL_PERIOD_S);
  }
  BSP_MicroUsbInit(Motor_CommandFeed, Motor_CommandReset);
  return BSP_CAN_Init(feedback_ids, MOTOR_CONTROL_COUNT, Motor_ReceiveFeedback);
}

/**
  * @brief  打包四路电流原始指令并发送 0x200 标准帧。
  * @param  currents: 四路有符号电流指令数组，当前控制流程将第四路置零。
  * @note   每路占两个字节，高字节在前；单位为电调协议原始值。
  * @retval BSP CAN 发送接口返回的 HAL 状态。
  */
static HAL_StatusTypeDef Motor_SendCurrents(const int16_t currents[4]) {
  uint8_t data[8];
  for (uint8_t i = 0; i < 4U; ++i) {
    uint16_t bits = (uint16_t)currents[i];
    data[2U * i] = (uint8_t)(bits >> 8);
    data[2U * i + 1U] = (uint8_t)bits;
  }
  return BSP_CAN_Send(0x200U, data, sizeof(data));
}

/**
  * @brief  解析电调反馈并累计转子编码器相对计数。
  * @param  header: 接收帧头。
  * @param  data: 8 字节反馈数据。
  * @note   在 CAN 接收中断中执行；仅处理 0x201～0x203 的标准数据帧。
  * @retval 无。
  */
static void Motor_ReceiveFeedback(const CAN_RxHeaderTypeDef *header,
                                  const uint8_t data[8]) {
  if (header->IDE != CAN_ID_STD || header->RTR != CAN_RTR_DATA ||
      header->StdId < 0x201U || header->StdId > 0x203U ||
      header->DLC != 8U) return;
  uint8_t i = (uint8_t)(header->StdId - 0x201U);
  uint16_t ecd = ((uint16_t)data[0] << 8) | data[1];
  if (ecd >= 8192U) return;
  uint32_t now = HAL_GetTick();
  if (!motor_measure[i].valid ||
      now - motor_measure[i].last_rx_ms >= MOTOR_FEEDBACK_TIMEOUT_MS) {
    // 反馈丢失后无法恢复遗漏的圈数，因此重新建立相对零点。
    motor_measure[i].relative_counts = 0;
    position_initialized[i] = 0U;
  } else {
    /* 相邻有效反馈的转子位移必须小于半圈，才能正确识别跨圈方向。 */
    int32_t delta = (int32_t)ecd - motor_measure[i].ecd;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    motor_measure[i].relative_counts += delta;
  }
  motor_measure[i].ecd = ecd;
  motor_measure[i].speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  motor_measure[i].given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  motor_measure[i].temperature = data[6];
  motor_measure[i].last_rx_ms = now;
  motor_measure[i].valid = 1U;
}

/**
  * @brief  读取指定电机的一致反馈快照。
  * @param  motor_id: 电调编号，取值为 1～3，对应数组下标 0～2。
  * @param  measure: 用于接收反馈快照的结构体指针。
  * @note   复制期间屏蔽中断，避免 64 位计数撕裂；成功不表示反馈在线。
  * @retval HAL_OK 表示复制成功，参数无效时返回 HAL_ERROR。
  */
HAL_StatusTypeDef Motor_GetMeasure(uint8_t motor_id,
                                    motor_measure_t *measure) {
  if (measure == NULL || motor_id < 1U || motor_id > MOTOR_CONTROL_COUNT) {
    return HAL_ERROR;
  }
  // 在临界区复制完整反馈，保证 64 位累计计数与其他字段一致。
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *measure = motor_measure[motor_id - 1U];
  __set_PRIMASK(primask);
  return HAL_OK;
}

/**
  * @brief  执行一次三路电机角度闭环并发送合并电流帧。
  * @param  pressed: 消抖后的按下事件；非零时为全部在线电机增加一个步进。
  * @note   仅由 TIM6 每 1 ms 调用一次；每路独立检测反馈超时和重建目标。
  * @retval 无。
  */
void Motor_ControlTick(uint8_t pressed) {
  int16_t steps[3];
  Motor_CommandTakeSteps(steps);
  int16_t currents[4] = {0};
  uint8_t abort_pending = 0U;
  for (uint8_t i = 0; i < MOTOR_CONTROL_COUNT; ++i) {
    motor_measure_t measure;
    Motor_GetMeasure(i + 1U, &measure);
    uint8_t healthy = measure.valid &&
        HAL_GetTick() - measure.last_rx_ms < MOTOR_FEEDBACK_TIMEOUT_MS &&
        !BSP_CAN_IsBusOff();
    if (!healthy) {
      if (motor_online[i]) abort_pending = 1U;
      motor_online[i] = 0U;
      position_initialized[i] = 0U;
      motor_current_command[i] = 0;
      Pid_Reset(&angle_pid[i]);
      angle_pid[i].kd = 0.0f;  // 离线时清除上一次指令方向对应的 D 项状态。
    } else {
      /* 累计转子计数除以减速比，得到多圈输出轴相对角度。 */
      motor_actual_angle_deg[i] = MOTOR_DIRECTION *
          (float)measure.relative_counts *
          (360.0f / (8192.0f * MOTOR_GEAR_RATIO));
      /* 首次在线或故障恢复时保持当前位置，不继续追踪旧目标。 */
      if (!position_initialized[i]) {
        motor_target_angle_deg[i] = motor_actual_angle_deg[i];
        Pid_Reset(&angle_pid[i]);
        angle_pid[i].kd = 0.0f;  // 重建位置参考时默认关闭 D 项。
        position_initialized[i] = 1U;
      }
      motor_online[i] = 1U;
      // 实体按键使全部在线电机的目标同时增加 10°。
      // USB 指令只作用于指定电机；离线电机的步进事件被丢弃。
      int32_t step_delta = (int32_t)steps[i] + (pressed ? 1 : 0);
      motor_target_angle_deg[i] += (float)step_delta * MOTOR_KEY_STEP_DEG;
      // 目标调整后保留该电机最近一次非零步进方向。
      // 目标增加时使用 PI；目标减少时启用配置的 D 项。
      if (step_delta < 0) {
        angle_pid[i].kd = MOTOR_ANGLE_KD;
      } else if (step_delta > 0) {
        angle_pid[i].kd = 0.0f;
      }
      // 即使 kd=0，PID 仍记录上一周期反馈；重新启用 D 时使用最新历史值。
      // 切换 D 项时保留比例参数和积分状态。
      // 角度 PID 直接计算目标电流；电流闭环由 C620 内部完成。
      // 微分作用于实际角度，目标阶跃不会直接引起微分冲击。
      // 将逻辑电流方向换回电调原始协议方向，保持闭环为负反馈。
      motor_current_command[i] = (int16_t)(MOTOR_DIRECTION * Pid_update(
          &angle_pid[i], motor_target_angle_deg[i], motor_actual_angle_deg[i]));
    }
    currents[i] = motor_current_command[i];
  }
  if (abort_pending) {
    // 取消旧合并帧，随后发送离线路电流置零的新控制帧。
    BSP_CAN_AbortPending();
  }
  // 三路电流指令合并在同一帧中，第四路保持为零。
  if (Motor_SendCurrents(currents) != HAL_OK) ++motor_tx_errors;
}
