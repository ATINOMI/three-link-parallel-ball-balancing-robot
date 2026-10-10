/**
  ******************************************************************************
  * @file    motor_telemetry.c
  * @brief   电机遥测：读取一致状态快照并通过 USB CDC 输出文本数据。
  ******************************************************************************
  */

/* 头文件包含 ------------------------------------------------*/
#include "motor_telemetry.h"

#include <stdio.h>

#include "motor_control.h"
#include "bsp_micro_usb.h"

/* 私有定义与变量 ------------------------------------------------*/
#define TELEMETRY_PERIOD_MS 100U  /* 遥测发送间隔，单位：毫秒。 */

typedef struct {
  motor_measure_t feedback;
  float target_deg;
  float angle_deg;
  int16_t current_command;
  uint8_t online;
} MotorSnapshot;

// 应用层格式化缓冲区；异步 USB 传输缓冲区由 BSP 单独持有。
static uint8_t tx_buffer[768];
static uint32_t last_send_ms;

/* 函数实现 ------------------------------------------------*/
/**
  * @brief  将角度格式化为保留两位小数的文本。
  * @param  text: 输出字符缓冲区。
  * @param  size: 输出缓冲区容量，单位为字节。
  * @param  degrees: 待格式化角度，单位为度。
  * @note   先换算为百分之一度再使用整数格式化，避免引入浮点 printf 库。
  * @retval 无。
  */
static void FormatDegrees(char *text, size_t size, float degrees) {
  // 使用整数格式化，避免链接 newlib-nano 的浮点 printf 实现。
  int64_t hundredths = (int64_t)(degrees * 100.0f +
                                 (degrees >= 0.0f ? 0.5f : -0.5f));
  const char *sign = hundredths < 0 ? "-" : "";
  if (hundredths < 0) hundredths = -hundredths;
  snprintf(text, size, "%s%ld.%02ld", sign,
           (long)(hundredths / 100), (long)(hundredths % 100));
}

/**
  * @brief  周期性发送三路电机状态文本。
  * @note   在主循环调用，每 100 ms 尝试发送；快照复制后再格式化，USB 未就绪时跳过。
  * @retval 无。
  */
void Motor_TelemetryPoll(void) {
  uint32_t now = HAL_GetTick();
  if (now - last_send_ms < TELEMETRY_PERIOD_MS) return;
  if (!BSP_MicroUsbReady()) return;

  MotorSnapshot snapshot[MOTOR_CONTROL_COUNT];
  // 仅在快照复制时屏蔽中断，格式化和发送均在主循环中完成。
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0; i < MOTOR_CONTROL_COUNT; ++i) {
    Motor_GetMeasure(i + 1U, &snapshot[i].feedback);
    snapshot[i].target_deg = motor_target_angle_deg[i];
    snapshot[i].angle_deg = motor_actual_angle_deg[i];
    snapshot[i].current_command = motor_current_command[i];
    snapshot[i].online = motor_online[i];
  }
  __set_PRIMASK(primask);

  /* 离线电机仍输出最近一次数据，接收端应根据 online 判断有效性。 */
  size_t used = 0;
  for (uint8_t i = 0; i < MOTOR_CONTROL_COUNT; ++i) {
    char target[32], angle[32];
    FormatDegrees(target, sizeof(target), snapshot[i].target_deg);
    FormatDegrees(angle, sizeof(angle), snapshot[i].angle_deg);
    int length = snprintf((char *)tx_buffer + used, sizeof(tx_buffer) - used,
        "t=%lu M%u online=%u target=%s angle=%s rpm=%d "
        "cmd=%d fb=%d temp=%u ecd=%u\r\n",
        (unsigned long)now, (unsigned)(i + 1U), (unsigned)snapshot[i].online,
        target, angle, (int)snapshot[i].feedback.speed_rpm,
        (int)snapshot[i].current_command, (int)snapshot[i].feedback.given_current,
        (unsigned)snapshot[i].feedback.temperature,
        (unsigned)snapshot[i].feedback.ecd);
    if (length < 0 || (size_t)length >= sizeof(tx_buffer) - used) return;
    used += (size_t)length;
  }

  if (BSP_MicroUsbSend(tx_buffer, (uint16_t)used)) {
    last_send_ms = now;
  }
}
