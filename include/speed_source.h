#ifndef SPEED_SOURCE_H
#define SPEED_SOURCE_H

#include "rs485_driver.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  RS485_Handle rs485;
  int16_t last_speed_rpm;
} SpeedSource;

void SpeedSource_Init(SpeedSource *source, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_id);

bool SpeedSource_Update(SpeedSource *source);

static inline int16_t SpeedSource_GetRpm(const SpeedSource *source) {
  return source->last_speed_rpm;
}

#endif
