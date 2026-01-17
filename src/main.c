#include "pid.h"
#include "speed_source.h"
#include "stm32f1xx_hal.h"

UART_HandleTypeDef huart5;

static SpeedSource speed_source;
static PID_Controller speed_pid;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);

static float PID_ClampTargetRpm(float rpm, float limit_rpm) {
  if (rpm > limit_rpm) {
    return limit_rpm;
  }
  if (rpm < -limit_rpm) {
    return -limit_rpm;
  }
  return rpm;
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_UART5_Init();

  SpeedSource_Init(&speed_source, &huart5, GPIOC, GPIO_PIN_12, 0x01);
  PID_Init(&speed_pid, 0.6f, 0.05f, 0.0f, -3000.0f, 3000.0f);

  uint32_t last_tick = HAL_GetTick();
  while (1) {
    uint32_t now = HAL_GetTick();
    float dt_s = (now - last_tick) / 1000.0f;
    if (dt_s <= 0.0f) {
      dt_s = 0.001f;
    }
    last_tick = now;

    if (SpeedSource_Update(&speed_source)) {
      float measured_rpm = (float)SpeedSource_GetRpm(&speed_source);
      float target_rpm = 1200.0f;
      float pid_output_rpm = PID_Update(&speed_pid, target_rpm, measured_rpm, dt_s);
      pid_output_rpm = PID_ClampTargetRpm(pid_output_rpm, 3000.0f);

      RS485_WriteTargetSpeedRpm(&speed_source.rs485, (int16_t)pid_output_rpm);
    }

    HAL_Delay(20);
  }
}

static void SystemClock_Config(void) {
  // TODO: use CubeMX generated clock configuration.
}

static void MX_GPIO_Init(void) {
  // TODO: configure RS485 DE pin (GPIOC PIN12) as push-pull output.
  // TODO: configure UART5 TX (PC12) / RX (PD2) pins as alternate function.
}

static void MX_UART5_Init(void) {
  // TODO: configure UART5 (baud, parity, stop bits) to match the driver.
}
