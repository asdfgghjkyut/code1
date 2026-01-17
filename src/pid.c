#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float output_min, float output_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output_min = output_min;
  pid->output_max = output_max;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s) {
  float error = setpoint - measurement;
  pid->integral += error * dt_s;
  float derivative = (error - pid->prev_error) / dt_s;

  float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
  if (output > pid->output_max) {
    output = pid->output_max;
  } else if (output < pid->output_min) {
    output = pid->output_min;
  }

  pid->prev_error = error;
  return output;
}
