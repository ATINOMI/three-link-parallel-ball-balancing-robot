/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.200 Hz 更新频率，而刚选的 20 Hz 是滤波截止频率
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_motor.h"
#include "gravity_compensation.h"
#include "pid.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "bsp_usb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_TELEMETRY_PERIOD_MS 5U
// 三路最终电流输出比例；上位机修改仅本次上电有效。
#define MOTOR_1_OUTPUT_SCALE 1.0f
#define MOTOR_2_OUTPUT_SCALE 0.9f
#define MOTOR_3_OUTPUT_SCALE 1.1f
#define SPEED_LOOP_PERIOD_MS 5U
// config/333motor-tuning-2026-11-01T07-16-08-871Z.json：每台电机的上升/下降默认PID。
#define MOTOR_1_PID_UP_KP 6.5f
#define MOTOR_1_PID_UP_KI 26.0f
#define MOTOR_1_PID_UP_KD 0.0f
#define MOTOR_1_PID_DOWN_KP 7.0f
#define MOTOR_1_PID_DOWN_KI 15.0f
#define MOTOR_1_PID_DOWN_KD 0.0f
#define MOTOR_2_PID_UP_KP 6.0f
#define MOTOR_2_PID_UP_KI 54.0f
#define MOTOR_2_PID_UP_KD 0.0f
#define MOTOR_2_PID_DOWN_KP 8.4f
#define MOTOR_2_PID_DOWN_KI 40.0f
#define MOTOR_2_PID_DOWN_KD 0.0f
#define MOTOR_3_PID_UP_KP 6.0f
#define MOTOR_3_PID_UP_KI 50.0f
#define MOTOR_3_PID_UP_KD 0.0f
#define MOTOR_3_PID_DOWN_KP 6.5f
#define MOTOR_3_PID_DOWN_KI 40.0f
#define MOTOR_3_PID_DOWN_KD 0.0f
// 角度外环输出单位deg/s；按config/333motor-tuning-2026-11-01T07-16-08-871Z.json固化，上电不启动角度环。
#define ANGLE_PID_KP 9.0f
#define ANGLE_PID_KI 2.0f
#define ANGLE_PID_KD 0.0f
#define ANGLE_MAX_SPEED_DPS 100.0f
#define ANGLE_MIN_SPEED_DPS 50.0f
#define MOTOR_3_ANGLE_PID_UP_KI 3.0f
#define MOTOR_3_ANGLE_MAX_SPEED_DPS 150.0f
#define MOTOR_3_ANGLE_MIN_SPEED_DPS 40.0f
#define ANGLE_SETTLE_ERROR_DEG 0.7f
#define ANGLE_SETTLE_TIME_MS 10U
#define SPEED_FRICTION_RAW 0
#define STATIC_FRICTION_REST_MS 20U
#define MOTOR_1_FRICTION_MOVE_MDEG 1000
#define MOTOR_2_FRICTION_MOVE_MDEG 1000
#define MOTOR_3_FRICTION_MOVE_MDEG 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
// 各电机首次反馈位置标定为 12.77 度，后续按编码器相对变化更新。
volatile float gravity_angle_deg[3] = {CAN_MOTOR_INITIAL_ANGLE_DEG,
                                      CAN_MOTOR_INITIAL_ANGLE_DEG,
                                      CAN_MOTOR_INITIAL_ANGLE_DEG};
// 正解得到的当前位姿；0.09 m 仅为牛顿迭代初值，不是运动目标。
volatile float gravity_pose[3] = {0.0f, 0.0f, 0.09f};
// 输出端重力补偿比例，可在调试器中修改。
volatile float k_g = 1.6f;
volatile float motor_output_scale[3] = {MOTOR_1_OUTPUT_SCALE,
                                      MOTOR_2_OUTPUT_SCALE,
                                      MOTOR_3_OUTPUT_SCALE};
// 所有允许重力补偿的模式：true 按实测上升补偿，false 持续补偿。
volatile bool gravity_upward_only_mode = true;
volatile float gravity_torque_cmd_nm[3];
volatile int16_t gravity_current_cmd[3];
// USB 起动增量/固定指令模式，共享所选电机与心跳；临界区内发布。
static volatile uint8_t control_mode = 1U, test_motor = 0U;
static volatile int16_t test_delta = 0, fixed_current = 0;
static volatile int8_t test_direction = 0;  // -1 抬升；+1 下降，仅表示测试增量方向。
static volatile uint32_t test_last_ms = 0;
static volatile int16_t sent_current[3], sent_requested_raw;
static volatile int32_t sent_base[3], sent_delta[3];
#define RX_CAPACITY 256U
static volatile uint8_t rx_buf[RX_CAPACITY];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_overflow;
// 速度单位为输出轴 deg/s；电流及摩擦前馈单位为 raw。
typedef struct { float kp, ki, kd; } SpeedPidGains;
// [电机][0上升/1下降]；每路仅保存一份PID状态，换向时复位。
static SpeedPidGains speed_pid_gains[3][2] = {
    {{MOTOR_1_PID_UP_KP, MOTOR_1_PID_UP_KI, MOTOR_1_PID_UP_KD},
     {MOTOR_1_PID_DOWN_KP, MOTOR_1_PID_DOWN_KI, MOTOR_1_PID_DOWN_KD}},
    {{MOTOR_2_PID_UP_KP, MOTOR_2_PID_UP_KI, MOTOR_2_PID_UP_KD},
     {MOTOR_2_PID_DOWN_KP, MOTOR_2_PID_DOWN_KI, MOTOR_2_PID_DOWN_KD}},
    {{MOTOR_3_PID_UP_KP, MOTOR_3_PID_UP_KI, MOTOR_3_PID_UP_KD},
     {MOTOR_3_PID_DOWN_KP, MOTOR_3_PID_DOWN_KI, MOTOR_3_PID_DOWN_KD}}};
static int8_t speed_pid_direction[3] = {-1, -1, -1};
static Pid_type speed_pid[3], angle_pid[3];
static SpeedPidGains angle_pid_gains[3][2] = {
    {{ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD}, {ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD}},
    {{ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD}, {ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD}},
    {{ANGLE_PID_KP, MOTOR_3_ANGLE_PID_UP_KI, ANGLE_PID_KD}, {ANGLE_PID_KP, ANGLE_PID_KI, ANGLE_PID_KD}}};
static int8_t angle_pid_direction[3] = {-1, -1, -1};
static volatile float angle_settle_error_deg = ANGLE_SETTLE_ERROR_DEG;
static volatile uint16_t angle_settle_time_ms = ANGLE_SETTLE_TIME_MS;
// 0纠偏、1范围内计时、2稳定到位；每路独立，判定周期1ms。
static volatile uint8_t angle_settle_state[3];
static uint32_t angle_settle_started_ms[3];
// 333motor-tuning-2026-11-01T07-16-08-871Z.json预设目标；上电仍为普通模式，收到A指令才运行角度环。
static volatile float angle_target[3] = {-70.0f,-30.0f,-30.0f};
static volatile float angle_max_speed[3] = {ANGLE_MAX_SPEED_DPS, ANGLE_MAX_SPEED_DPS,
                                          MOTOR_3_ANGLE_MAX_SPEED_DPS};
static volatile float angle_min_speed[3] = {ANGLE_MIN_SPEED_DPS, ANGLE_MIN_SPEED_DPS,
                                          MOTOR_3_ANGLE_MIN_SPEED_DPS};
static volatile uint16_t speed_period_ms = SPEED_LOOP_PERIOD_MS;
static volatile uint16_t filter_period_ms = CAN_SPEED_FILTER_PERIOD_MS;
static volatile uint8_t speed_divider, filter_divider;
static volatile float speed_target[3], speed_measured[3];
static volatile int16_t speed_pid_cmd[3], speed_friction_cmd[3];
static volatile int32_t friction_last_encoder[3], friction_start_encoder[3];
static volatile int32_t friction_move_mdeg[3] = {MOTOR_1_FRICTION_MOVE_MDEG,
                                               MOTOR_2_FRICTION_MOVE_MDEG,
                                               MOTOR_3_FRICTION_MOVE_MDEG};
static uint32_t friction_last_motion_ms[3];
static volatile int8_t friction_direction[3];
static volatile bool friction_starting[3];
static volatile int16_t motor_friction_raw[3] = {SPEED_FRICTION_RAW,
                                               SPEED_FRICTION_RAW,
                                               SPEED_FRICTION_RAW};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t ScaleCurrent(int32_t raw, float scale) {
  float value = raw * scale;
  return (int32_t)(value + (value < 0.0f ? -0.5f : 0.5f));
}

static void LoadSpeedPidGains(int motor) {
  const SpeedPidGains *g = &speed_pid_gains[motor][speed_pid_direction[motor] > 0];
  speed_pid[motor].kp = g->kp; speed_pid[motor].ki = g->ki; speed_pid[motor].kd = g->kd;
}

static void LoadAnglePidGains(int motor) {
  const SpeedPidGains *g = &angle_pid_gains[motor][angle_pid_direction[motor] > 0];
  angle_pid[motor].kp = g->kp; angle_pid[motor].ki = g->ki; angle_pid[motor].kd = g->kd;
}

void BSP_USB_ReceiveCallback(const uint8_t *data, uint32_t length) {
  for (uint32_t i = 0; i < length; ++i) {
    uint16_t next = (uint16_t)((rx_head + 1U) % RX_CAPACITY);
    if (next == rx_tail) { rx_overflow = 1U; continue; }
    rx_buf[rx_head] = data[i]; rx_head = next;
  }
}

static void HandleUsbLine(char *line, uint32_t now) {
  int a, b, c, d, e, f, g = 0, consumed = 0;
  if (sscanf(line, "P,%d%n", &a, &consumed) == 1 &&
      line[consumed] == '\0' && a >= 0 && a <= 5000) {
    k_g = (float)a / 1000.0f;
  } else if (sscanf(line, "R,%d,%d,%d%n", &a, &b, &c, &consumed) == 3 &&
             line[consumed] == '\0' && a >= 0 && a <= 5000 &&
             b >= 0 && b <= 5000 && c >= 0 && c <= 5000) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    motor_output_scale[0] = a / 1000.0f;
    motor_output_scale[1] = b / 1000.0f;
    motor_output_scale[2] = c / 1000.0f;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "U,%d,%d%n", &a, &b, &consumed) == 2 &&
             line[consumed] == '\0' && a >= 1 && a <= 50 && b >= 1 && b <= 20) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    speed_period_ms = (uint16_t)a; filter_period_ms = (uint16_t)b;
    speed_divider = 0U; filter_divider = 0U;
    CAN_SetSpeedFilterPeriod(filter_period_ms);
    for (int i = 0; i < 3; ++i) {
      speed_pid[i].dt = a * 0.001f; Pid_Reset(&speed_pid[i]); speed_pid_cmd[i] = 0;
      angle_pid[i].dt = a * 0.001f; Pid_Reset(&angle_pid[i]); angle_settle_state[i] = 0U;
      if (control_mode == 6U) speed_target[i] = 0.0f;
    }
    __set_PRIMASK(mask);
  } else if (sscanf(line, "H,%d,%d%n", &a, &b, &consumed) == 2 &&
             line[consumed] == '\0' && a >= 1 && a <= 10000 && b >= 1 && b <= 5000) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    angle_settle_error_deg = a / 1000.0f; angle_settle_time_ms = (uint16_t)b;
    for (int i = 0; i < 3; ++i) angle_settle_state[i] = 0U;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "J,%d,%d,%d,%d,%d,%d%n,%d%n", &a, &b, &c, &d, &e, &f,
                    &consumed, &g, &consumed) >= 6 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 && (b == -1 || b == 1) &&
             c >= 0 && d >= 0 && e >= 0 && f >= 1 && f <= 180000 && g >= 0 && g <= f) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    angle_pid_gains[a - 1][b > 0] = (SpeedPidGains){c / 1000.0f, d / 1000.0f, e / 1000.0f};
    bool reset = angle_pid_direction[a - 1] == b || angle_max_speed[a - 1] != f / 1000.0f ||
                 angle_min_speed[a - 1] != g / 1000.0f;
    angle_min_speed[a - 1] = g / 1000.0f;  // 旧J未带下限时为0，保留旧行为。
    angle_max_speed[a - 1] = f / 1000.0f;
    angle_pid[a - 1].max_output = angle_pid[a - 1].max_ki_output = angle_max_speed[a - 1];
    if (angle_pid_direction[a - 1] == b) LoadAnglePidGains(a - 1);
    if (reset) {
      Pid_Reset(&angle_pid[a - 1]); angle_settle_state[a - 1] = 0U;
      if (control_mode == 6U) { speed_target[a - 1] = 0.0f; speed_pid_cmd[a - 1] = 0; }
    }
    __set_PRIMASK(mask);
  } else if (sscanf(line, "K,%d,%d,%d,%d,%d,%d%n", &a, &b, &c, &d, &e, &f, &consumed) == 6 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 && (b == -1 || b == 1) &&
             c >= 0 && d >= 0 && e >= 0 && f >= 0 && f <= 16384) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    speed_pid_gains[a - 1][b > 0] = (SpeedPidGains){c / 1000.0f, d / 1000.0f, e / 1000.0f};
    motor_friction_raw[a - 1] = (int16_t)f;
    if (speed_pid_direction[a - 1] == b) {
      LoadSpeedPidGains(a - 1); Pid_Reset(&speed_pid[a - 1]); speed_pid_cmd[a - 1] = 0;
    }
    __set_PRIMASK(mask);
  } else if (sscanf(line, "K,%d,%d,%d,%d,%d%n", &a, &b, &c, &d, &e, &consumed) == 5 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 &&
             b >= 0 && c >= 0 && d >= 0 && e >= 0 && e <= 16384) {
    // 兼容旧上位机：旧K指令同时设置上下两组。
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    speed_pid_gains[a - 1][0] = speed_pid_gains[a - 1][1] =
        (SpeedPidGains){b / 1000.0f, c / 1000.0f, d / 1000.0f};
    motor_friction_raw[a - 1] = (int16_t)e;
    LoadSpeedPidGains(a - 1); Pid_Reset(&speed_pid[a - 1]); speed_pid_cmd[a - 1] = 0;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "T,%d,%d%n", &a, &b, &consumed) == 2 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 && b >= 1 && b <= 1000) {
    friction_move_mdeg[a - 1] = b;
  } else if (sscanf(line, "B,%d%n", &a, &consumed) == 1 &&
             line[consumed] == '\0' && a >= 1 && a <= 3) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (control_mode >= 5U && speed_target[a - 1] != 0.0f) {
      friction_last_encoder[a - 1] = CAN_GetMotorMeasure(a)->total_ecd;
      friction_start_encoder[a - 1] = friction_last_encoder[a - 1];
      friction_direction[a - 1] = speed_target[a - 1] > 0.0f ? 1 : -1;
      friction_starting[a - 1] = true;
    }
    __set_PRIMASK(mask);
  } else if (sscanf(line, "A,%d,%d,%d%n", &a, &b, &c, &consumed) == 3 &&
             line[consumed] == '\0' && a >= -180000 && a <= 180000 &&
             b >= -180000 && b <= 180000 && c >= -180000 && c <= 180000) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (control_mode != 6U) {
      for (int i = 0; i < 3; ++i) {
        Pid_Reset(&angle_pid[i]); Pid_Reset(&speed_pid[i]); angle_settle_state[i] = 0U;
        speed_target[i] = 0.0f; speed_pid_cmd[i] = 0;
      }
    }
    const float target[3] = {a / 1000.0f, b / 1000.0f, c / 1000.0f};
    for (int i = 0; i < 3; ++i) {
      if (angle_target[i] != target[i]) angle_settle_state[i] = 0U;
      angle_target[i] = target[i];
    }
    test_motor = 0U; test_delta = 0; fixed_current = 0; test_direction = 0;
    test_last_ms = now; control_mode = 6U;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "V,%d,%d,%d%n", &a, &b, &c, &consumed) == 3 &&
             line[consumed] == '\0' && a >= -180000 && a <= 180000 &&
             b >= -180000 && b <= 180000 && c >= -180000 && c <= 180000) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (control_mode != 5U) {
      for (int i = 0; i < 3; ++i) Pid_Reset(&speed_pid[i]);
    }
    speed_target[0] = a / 1000.0f;
    speed_target[1] = b / 1000.0f;
    speed_target[2] = c / 1000.0f;
    test_motor = 0U; test_delta = 0; fixed_current = 0; test_direction = 0;
    test_last_ms = now; control_mode = 5U;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "D,%d,%d,%d%n", &a, &b, &c, &consumed) == 3 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 && (b == -1 || b == 1) &&
             c >= -16384 && c <= 16384 && b * c >= 0) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    test_motor = (uint8_t)a; test_direction = (int8_t)b;
    test_delta = (int16_t)c; fixed_current = 0; test_last_ms = now; control_mode = 2U;
    __set_PRIMASK(mask);
  } else if (sscanf(line, "F,%d,%d,%d%n", &a, &b, &c, &consumed) == 3 &&
             line[consumed] == '\0' && a >= 1 && a <= 3 && (b == 0 || b == 1) &&
             c >= -16384 && c <= 16384) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    test_motor = (uint8_t)a; fixed_current = (int16_t)c;
    test_delta = 0; test_direction = 0; test_last_ms = now; control_mode = b ? 4U : 3U;
    __set_PRIMASK(mask);
  } else if (strcmp(line, "S") == 0 || strcmp(line, "G") == 0) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    test_motor = 0U; test_delta = 0; fixed_current = 0; test_direction = 0;
    control_mode = line[0] == 'S' ? 0U : 1U;
    __set_PRIMASK(mask);
  }
}

static void ProcessUsb(uint32_t now) {
  static char line[64]; static uint8_t used, discard;
  if (rx_overflow) {
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    rx_tail = rx_head; rx_overflow = 0U; __set_PRIMASK(mask);
    used = 0U; discard = 1U;
  }
  while (rx_tail != rx_head) {
    char c = (char)rx_buf[rx_tail]; rx_tail = (uint16_t)((rx_tail + 1U) % RX_CAPACITY);
    if (c == '\n') {
      if (!discard) { if (used && line[used - 1U] == '\r') --used; line[used] = '\0'; HandleUsbLine(line, now); }
      used = 0U; discard = 0U;
    } else if (!discard) {
      if (used < sizeof(line) - 1U) line[used++] = c; else discard = 1U;
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  if (CAN_MotorStart() != HAL_OK) {
    Error_Handler();
  }
  for (int i = 0; i < 3; ++i) {
    const SpeedPidGains *g = &speed_pid_gains[i][0];
    Pid_Init(&speed_pid[i], g->kp, g->ki, g->kd,
             16384.0f, 16384.0f, speed_period_ms * 0.001f);
    g = &angle_pid_gains[i][0];
    Pid_Init(&angle_pid[i], g->kp, g->ki, g->kd,
             angle_max_speed[i], angle_max_speed[i], speed_period_ms * 0.001f);
  }
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_compute_ms = 0;
    static uint32_t last_send_ms = 0;
    static char usb_tx_buffer[1280];
    uint32_t now = HAL_GetTick();
    ProcessUsb(now);
    if ((uint32_t)(now - last_compute_ms) >= 5U) {
      last_compute_ms = now;
      float theta_rad[3];
      for (int i = 0; i < 3; ++i) {
        gravity_angle_deg[i] = CAN_GetMotorAngle(i + 1U);
        theta_rad[i] = gravity_angle_deg[i] * 0.01745329252f;
      }
      float pose[3] = {gravity_pose[0], gravity_pose[1], gravity_pose[2]};
      float torque_nm[3];
      int16_t current_cmd[3];
      GravityCompensation(theta_rad, pose, k_g, torque_nm, current_cmd);
      const bool upward_only = gravity_upward_only_mode;
      for (int i = 0; i < 3; ++i) {
        // 实际角度窗口测速：上升为负；下降和静止不加重力补偿。
        if (upward_only &&
            CAN_GetMotorMeasure(i + 1U)->angle_speed_rpm >= 0.0f) {
          torque_nm[i] = 0.0f;
          current_cmd[i] = 0;
        }
        gravity_pose[i] = pose[i];
        gravity_torque_cmd_nm[i] = torque_nm[i];
      }
      // 一次发布三路指令，避免定时中断取到不同计算周期的数据。
      uint32_t primask = __get_PRIMASK();
      __disable_irq();
      for (int i = 0; i < 3; ++i) {
        gravity_current_cmd[i] = current_cmd[i];
      }
      __set_PRIMASK(primask);
    }
    if ((uint32_t)(now - last_send_ms) >= USB_TELEMETRY_PERIOD_MS && BSP_USB_CanSend()) {
      last_send_ms = now;
      // 发送缓冲区保持不变直到 CDC 完成；快照避免中断更新时混读。
      int32_t angle[3], pose[3], scale[3], base[3], delta[3], speed_data[80];
      int16_t cmd[3], fb[3], requested_raw;
      uint8_t mode, motor, fresh = 0U;
      int32_t kg;
      uint32_t mask = __get_PRIMASK(); __disable_irq();
      mode = control_mode; motor = test_motor; requested_raw = sent_requested_raw;
      kg = (int32_t)(k_g * 1000.0f + 0.5f);
      speed_data[72] = ScaleCurrent(1000, angle_settle_error_deg);
      speed_data[73] = angle_settle_time_ms;
      speed_data[27] = speed_period_ms;
      speed_data[28] = filter_period_ms;
      speed_data[29] = (int32_t)(CAN_SPEED_FILTER_CUTOFF_HZ * 1000.0f + 0.5f);
      for (int i = 0; i < 3; ++i) {
        speed_data[i] = ScaleCurrent(1000, speed_measured[i]);
        speed_data[3 + i] = ScaleCurrent(1000, speed_target[i]);
        speed_data[6 + 4 * i] = ScaleCurrent(1000, speed_pid[i].kp);
        speed_data[7 + 4 * i] = ScaleCurrent(1000, speed_pid[i].ki);
        speed_data[8 + 4 * i] = ScaleCurrent(1000, speed_pid[i].kd);
        speed_data[9 + 4 * i] = motor_friction_raw[i];
        speed_data[18 + i] = speed_pid_cmd[i];
        speed_data[21 + i] = speed_friction_cmd[i];
        speed_data[24 + i] = friction_move_mdeg[i];
        speed_data[48 + i] = ScaleCurrent(1000, angle_target[i]);
        speed_data[74 + i] = angle_settle_state[i];
        speed_data[77 + i] = ScaleCurrent(1000, angle_min_speed[i]);
        speed_data[69 + i] = ScaleCurrent(1000, angle_max_speed[i]);
        for (int direction = 0; direction < 2; ++direction) {
          const SpeedPidGains *g = &speed_pid_gains[i][direction];
          int offset = 30 + 6 * i + 3 * direction;
          speed_data[offset] = ScaleCurrent(1000, g->kp);
          speed_data[offset + 1] = ScaleCurrent(1000, g->ki);
          speed_data[offset + 2] = ScaleCurrent(1000, g->kd);
          g = &angle_pid_gains[i][direction];
          offset = 51 + 6 * i + 3 * direction;
          speed_data[offset] = ScaleCurrent(1000, g->kp);
          speed_data[offset + 1] = ScaleCurrent(1000, g->ki);
          speed_data[offset + 2] = ScaleCurrent(1000, g->kd);
        }
        scale[i] = (int32_t)(motor_output_scale[i] * 1000.0f + 0.5f);
        angle[i] = (int32_t)(gravity_angle_deg[i] * 1000.0f);
        pose[i] = (int32_t)(gravity_pose[i] * (i == 2 ? 1000000.0f : 1000.0f));
        base[i] = sent_base[i]; delta[i] = sent_delta[i];
        cmd[i] = sent_current[i]; fb[i] = CAN_GetMotorMeasure(i + 1U)->given_cuurent;
        if (CAN_MotorFeedbackIsFresh(i + 1U, 100U)) fresh |= (uint8_t)(1U << i);
      }
      __set_PRIMASK(mask);
      // V11: 保留V10的105字段，追加三路角度环最小速度(毫度/秒)，共108字段。
      int length = snprintf(usb_tx_buffer, sizeof(usb_tx_buffer),
          "V11,%lu,%u,%ld,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,"
          "%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%d",
          (unsigned long)now, mode, (long)kg, motor, fresh,
          (long)angle[0], (long)angle[1], (long)angle[2],
          (long)pose[0], (long)pose[1], (long)pose[2],
          (long)base[0], (long)base[1], (long)base[2],
          (long)delta[0], (long)delta[1], (long)delta[2],
          cmd[0], cmd[1], cmd[2], fb[0], fb[1], fb[2],
          (long)scale[0], (long)scale[1], (long)scale[2], requested_raw);
      for (int i = 0; i < 80; ++i) {
        length += snprintf(usb_tx_buffer + length, sizeof(usb_tx_buffer) - length,
                           ",%ld", (long)speed_data[i]);
      }
      usb_tx_buffer[length++] = '\r'; usb_tx_buffer[length++] = '\n';
      if (length > 0 && length < (int)sizeof(usb_tx_buffer))
        BSP_USB_Send((const uint8_t *)usb_tx_buffer, (uint16_t)length);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Blue_LED_GPIO_Port, Blue_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blue_LED_Pin */
  GPIO_InitStruct.Pin = Blue_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blue_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Key_Pin */
  GPIO_InitStruct.Pin = Key_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Key_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    int16_t current_cmd[3] = {0, 0, 0};
    // 可达范围由上位机严格三维逆解判定；保留失联急停和电流限幅。
    uint8_t fresh = CAN_MotorFeedbackIsFresh(1U, 100U) &&
                    CAN_MotorFeedbackIsFresh(2U, 100U) &&
                    CAN_MotorFeedbackIsFresh(3U, 100U);
    // 心跳超时或反馈中断时撤销测试指令，恢复原控制逻辑。
    if (control_mode >= 2U &&
        (!fresh || (uint32_t)(HAL_GetTick() - test_last_ms) > 300U)) {
      control_mode = control_mode >= 5U ? 0U : 1U;
      test_motor = 0U; test_delta = 0; fixed_current = 0; test_direction = 0;
    }
    // 1kHz电流发送不变，PID和滤波器分别分频，先滤波后计算PID。
    if (++filter_divider >= filter_period_ms) {
      filter_divider = 0U; CAN_FilterSpeeds();
    }
    bool speed_step = ++speed_divider >= speed_period_ms;
    if (speed_step) speed_divider = 0U;
    for (int i = 0; i < 3; ++i) {
      const volatile motor_measure_t *motor = CAN_GetMotorMeasure(i + 1U);
      speed_measured[i] = motor->filtered_speed_rpm * (6.0f / CAN_MOTOR_REDUCTION_RATIO);
      uint32_t now = HAL_GetTick();
      // 连续处于范围内才到位；经过目标但未停留不能算成功。
      if (control_mode == 6U && fresh) {
        float angle = CAN_GetMotorAngle(i + 1U), error = angle_target[i] - angle;
        if (error >= -angle_settle_error_deg && error <= angle_settle_error_deg) {
          if (angle_settle_state[i] == 0U) {
            angle_settle_state[i] = 1U; angle_settle_started_ms[i] = now;
          }
          if (angle_settle_state[i] == 1U &&
              (uint32_t)(now - angle_settle_started_ms[i]) >= angle_settle_time_ms) {
            angle_settle_state[i] = 2U; speed_pid_cmd[i] = 0;
          }
        } else angle_settle_state[i] = 0U;
        if (angle_settle_state[i] == 2U) {
          Pid_Reset(&angle_pid[i]); speed_target[i] = 0.0f;
        } else if (speed_step) {
          int8_t direction = error > 0.0f ? 1 : error < 0.0f ? -1 : angle_pid_direction[i];
          if (direction != angle_pid_direction[i]) {
            angle_pid_direction[i] = direction; LoadAnglePidGains(i); Pid_Reset(&angle_pid[i]);
          }
          speed_target[i] = Pid_update(&angle_pid[i], angle_target[i], angle);
          // 只在到位范围外抬高非零速度；确认期间沿用PID，稳定到位后仍为0。
          if (angle_settle_state[i] == 0U) {
            float minimum = angle_min_speed[i];
            if (speed_target[i] > 0.0f && speed_target[i] < minimum) speed_target[i] = minimum;
            else if (speed_target[i] < 0.0f && speed_target[i] > -minimum) speed_target[i] = -minimum;
          }
        }
      } else { Pid_Reset(&angle_pid[i]); angle_settle_state[i] = 0U; }
      // 直接检查相对起点的净位移；达到阈值后，在本次1 ms输出中撤销。
      int32_t encoder = motor->total_ecd;
      float limit = friction_move_mdeg[i] * 0.001f;
      const float count_to_deg = 360.0f / (8192.0f * CAN_MOTOR_REDUCTION_RATIO);
      bool moved = encoder != friction_last_encoder[i];
      if (moved) friction_last_motion_ms[i] = now;
      friction_last_encoder[i] = encoder;
      int8_t direction = control_mode >= 5U && fresh ?
                        (speed_target[i] > 0.0f) - (speed_target[i] < 0.0f) : 0;
      if (direction != friction_direction[i]) {
        friction_starting[i] = direction != 0 &&
            (uint32_t)(now - friction_last_motion_ms[i]) >= STATIC_FRICTION_REST_MS;
        friction_direction[i] = direction;
        friction_start_encoder[i] = encoder;
      }
      float displacement = (encoder - friction_start_encoder[i]) * count_to_deg;
      if (displacement >= limit || displacement <= -limit || direction == 0)
        friction_starting[i] = false;
      speed_friction_cmd[i] = friction_starting[i] ? direction * motor_friction_raw[i] : 0;
      if (control_mode >= 5U && fresh) {
        // 目标负速度用上升组，正速度用下降组；零目标沿用上一组。
        int8_t pid_direction = speed_target[i] > 0.0f ? 1 :
                               speed_target[i] < 0.0f ? -1 : speed_pid_direction[i];
        if (pid_direction != speed_pid_direction[i]) {
          speed_pid_direction[i] = pid_direction;
          LoadSpeedPidGains(i); Pid_Reset(&speed_pid[i]); speed_pid_cmd[i] = 0;
        }
        if (speed_step) speed_pid_cmd[i] = (int16_t)ScaleCurrent(1,
            Pid_update(&speed_pid[i], speed_target[i], speed_measured[i]));
      } else {
        Pid_Reset(&speed_pid[i]); speed_target[i] = 0.0f;
        speed_pid_cmd[i] = 0; speed_friction_cmd[i] = 0;
      }
    }
    for (int i = 0; i < 3; ++i) {
      sent_base[i] = 0; sent_delta[i] = 0;
      if (fresh && control_mode != 0U) {
        int32_t base = gravity_current_cmd[i];
        // 重力补偿只看实测角度窗口速度，不看目标速度/电流方向。
        if (test_motor == i + 1U && control_mode == 3U) base = 0;
        if (base > 16384) base = 16384;
        if (base < -16384) base = -16384;
        int32_t total = base;
        if (test_motor == i + 1U) {
          if (control_mode == 2U) total += test_delta;
          else if (control_mode == 3U || control_mode == 4U) total += fixed_current;
        }
        if (control_mode >= 5U) total += speed_pid_cmd[i] + speed_friction_cmd[i];
        float scale = motor_output_scale[i];
        base = ScaleCurrent(base, scale);
        total = ScaleCurrent(total, scale);
        if (total > 16384) total = 16384;
        if (total < -16384) total = -16384;
        if (base > 16384) base = 16384;
        if (base < -16384) base = -16384;
        sent_base[i] = base;
        sent_delta[i] = total - base;
        current_cmd[i] = (int16_t)total;
      }
      sent_current[i] = current_cmd[i];
    }
    sent_requested_raw = control_mode == 2U ? test_delta :
                         ((control_mode == 3U || control_mode == 4U) ? fixed_current : 0);
    CAN_cmd_three(current_cmd);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
