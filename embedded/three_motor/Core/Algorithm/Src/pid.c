#include "pid.h"

static float Clamp(float value, float limit) {
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void Pid_Reset(Pid_type *pid) {
  pid->setpoint = 0.0f; pid->feedback = 0.0f; pid->error = 0.0f;
  pid->p_output = 0.0f; pid->i_output = 0.0f; pid->d_output = 0.0f;
  pid->output = 0.0f; pid->previous_error = 0.0f; pid->initialized = false;
}

void Pid_Init(Pid_type *pid, float kp, float ki, float kd,
              float max_output, float max_ki_output, float dt) {
  pid->kp = kp; pid->ki = ki; pid->kd = kd;
  pid->max_output = max_output; pid->max_ki_output = max_ki_output; pid->dt = dt;
  Pid_Reset(pid);
}

float Pid_update(Pid_type *pid, float setpoint, float feedback) {
  float error = setpoint - feedback;
  float integral = Clamp(pid->i_output + pid->ki * error * pid->dt,
                         pid->max_ki_output);
  pid->p_output = pid->kp * error;
  pid->d_output = pid->initialized ? pid->kd *
                 (error - pid->previous_error) / pid->dt : 0.0f;
  float candidate = pid->p_output + integral + pid->d_output;
  if (!((candidate > pid->max_output && error > 0.0f) ||
        (candidate < -pid->max_output && error < 0.0f))) pid->i_output = integral;
  pid->output = Clamp(pid->p_output + pid->i_output + pid->d_output,
                      pid->max_output);
  pid->setpoint = setpoint; pid->feedback = feedback; pid->error = error;
  pid->previous_error = error; pid->initialized = true;
  return pid->output;
}
