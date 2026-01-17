#ifndef RS485_DRIVER_H
#define RS485_DRIVER_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define RS485_MAX_FRAME 32u

typedef struct {
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t slave_id;
} RS485_Handle;

void RS485_Init(RS485_Handle *handle, UART_HandleTypeDef *huart,
                GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_id);

bool RS485_ReadSpeedRpm(RS485_Handle *handle, int16_t *speed_rpm);
bool RS485_WriteTargetSpeedRpm(RS485_Handle *handle, int16_t target_rpm);

#endif
