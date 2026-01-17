#include "speed_source.h"

void SpeedSource_Init(SpeedSource *source, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_id) {
  RS485_Init(&source->rs485, huart, de_port, de_pin, slave_id);
  source->last_speed_rpm = 0;
}

bool SpeedSource_Update(SpeedSource *source) {
  int16_t speed_rpm = 0;
  if (!RS485_ReadSpeedRpm(&source->rs485, &speed_rpm)) {
    return false;
  }
  source->last_speed_rpm = speed_rpm;
  return true;
}
