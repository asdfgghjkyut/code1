#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float output_min;
  float output_max;
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float output_min, float output_max);
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s);

#endif
