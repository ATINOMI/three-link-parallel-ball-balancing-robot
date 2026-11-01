
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "pid.h"
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
typedef struct{int Instance;}TIM_HandleTypeDef;
#define TIM6 6
static uint32_t tick;
static uint8_t fresh_mask=7;
static int16_t out[3];
static uint32_t HAL_GetTick(void){return tick;}
static int CAN_MotorFeedbackIsFresh(unsigned id,unsigned timeout){(void)timeout;return fresh_mask&(1U<<(id-1));}
static void CAN_cmd_three(const int16_t *v){memcpy(out,v,sizeof(out));}
typedef struct
{
    int32_t total_ecd;          // 相对首次反馈/设零位置的累计编码器计数。
    uint16_t ecd;               //转子机械角度
    float angle_speed_rpm;      // 角度窗口测速，转子 rpm。
    float filtered_speed_rpm;   // 角度测速低通后的转子 rpm，供 PID 使用。
    int16_t speed_rpm;          //电调原始转子转速，仅供观察，不参与闭环
    int16_t given_cuurent;      //实际转矩电流
    uint8_t temperate;          //电机温度
    int16_t last_ecd;           //上次转子机械角度
}motor_measure_t;
#define CAN_MOTOR_INITIAL_ANGLE_DEG 12.77f
#define CAN_MOTOR_REDUCTION_RATIO (3591.0f / 187.0f)
static motor_measure_t measured[3];
static const volatile motor_measure_t *CAN_GetMotorMeasure(unsigned id){return &measured[id-1];}
static float angles[3];
static float CAN_GetMotorAngle(unsigned id){return angles[id-1];}

// 三路最终电流输出比例；上位机修改仅本次上电有效。
#define SPEED_PID_UP_KP 0.0f
#define SPEED_PID_UP_KI 0.0f
#define SPEED_PID_UP_KD 0.0f
#define SPEED_PID_DOWN_KP 0.0f
#define SPEED_PID_DOWN_KI 0.0f
#define SPEED_PID_DOWN_KD 0.0f


#include <math.h>
#include "butterworth.h"
#define STATIC_FRICTION_MOVE_MDEG 50
#define CAN_MOTOR_COUNT 3U
#define CAN_SPEED_WINDOW_MS 5U
#define CAN_SPEED_FILTER_PERIOD_MS 5U
#define CAN_SPEED_FILTER_CUTOFF_HZ 20.0f
typedef struct {
  int32_t  delta_sum;  // 当前测速窗口内累计的编码器增量
  uint32_t last_ms;    // 上次更新速度的时间
  ButterworthState lowpass;
} SpeedFilterState;
static SpeedFilterState speed_filter[3];
static ButterworthCoefficients speed_filter_coefficients;
static unsigned filter_calls;
static char wire[1280];
static int BSP_USB_CanSend(void){return 1;}
static void BSP_USB_Send(const uint8_t *p,uint16_t length){assert(length<sizeof(wire));memcpy(wire,p,length);wire[length]=0;}
void CAN_SetSpeedFilterPeriod(uint16_t period_ms) {
  Butterworth_Design(&speed_filter_coefficients, CAN_SPEED_FILTER_CUTOFF_HZ, period_ms * 0.001f);
  for (uint8_t i = 0; i < CAN_MOTOR_COUNT; ++i)
    Butterworth_Reset(&speed_filter[i].lowpass, &speed_filter_coefficients, measured[i].filtered_speed_rpm);
}

static void Actual_CAN_FilterSpeeds(void) {
  for (uint8_t i = 0; i < CAN_MOTOR_COUNT; ++i)
    measured[i].filtered_speed_rpm = Butterworth_Update(
        &speed_filter[i].lowpass, &speed_filter_coefficients, measured[i].angle_speed_rpm);
}


static void CAN_FilterSpeeds(void){filter_calls++;Actual_CAN_FilterSpeeds();}





// 三路最终电流输出比例；上位机修改仅本次上电有效。
// final parameter.json：每台电机的上升/下降默认PID。
// 角度外环输出单位deg/s；默认不主动产生目标速度。
