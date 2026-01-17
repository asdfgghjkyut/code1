#include "rs485_driver.h"

static uint16_t RS485_Crc16(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFFu;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1u) {
        crc = (crc >> 1) ^ 0xA001u;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void RS485_SetTx(const RS485_Handle *handle) {
  HAL_GPIO_WritePin(handle->de_port, handle->de_pin, GPIO_PIN_SET);
}

static void RS485_SetRx(const RS485_Handle *handle) {
  HAL_GPIO_WritePin(handle->de_port, handle->de_pin, GPIO_PIN_RESET);
}

void RS485_Init(RS485_Handle *handle, UART_HandleTypeDef *huart,
                GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t slave_id) {
  handle->huart = huart;
  handle->de_port = de_port;
  handle->de_pin = de_pin;
  handle->slave_id = slave_id;
  RS485_SetRx(handle);
}

bool RS485_ReadSpeedRpm(RS485_Handle *handle, int16_t *speed_rpm) {
  uint8_t tx[8] = {0};
  uint8_t rx[7] = {0};

  tx[0] = handle->slave_id;
  tx[1] = 0x03;            // Modbus Read Holding Registers
  tx[2] = 0x00;            // register high byte
  tx[3] = 0x20;            // register low byte (0x0020 speed register)
  tx[4] = 0x00;            // number of registers high
  tx[5] = 0x01;            // number of registers low
  uint16_t crc = RS485_Crc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFFu);
  tx[7] = (uint8_t)(crc >> 8);

  RS485_SetTx(handle);
  if (HAL_UART_Transmit(handle->huart, tx, sizeof(tx), 10) != HAL_OK) {
    RS485_SetRx(handle);
    return false;
  }
  RS485_SetRx(handle);

  if (HAL_UART_Receive(handle->huart, rx, sizeof(rx), 20) != HAL_OK) {
    return false;
  }

  uint16_t rx_crc = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
  if (RS485_Crc16(rx, 5) != rx_crc) {
    return false;
  }

  if (rx[0] != handle->slave_id || rx[1] != 0x03 || rx[2] != 0x02) {
    return false;
  }

  *speed_rpm = (int16_t)((rx[3] << 8) | rx[4]);
  return true;
}

bool RS485_WriteTargetSpeedRpm(RS485_Handle *handle, int16_t target_rpm) {
  uint8_t tx[8] = {0};
  uint8_t rx[8] = {0};

  tx[0] = handle->slave_id;
  tx[1] = 0x06;            // Modbus Write Single Register
  tx[2] = 0x00;            // register high byte
  tx[3] = 0x00;            // register low byte (0x0000 target speed)
  tx[4] = (uint8_t)(target_rpm >> 8);
  tx[5] = (uint8_t)(target_rpm & 0xFF);
  uint16_t crc = RS485_Crc16(tx, 6);
  tx[6] = (uint8_t)(crc & 0xFFu);
  tx[7] = (uint8_t)(crc >> 8);

  RS485_SetTx(handle);
  if (HAL_UART_Transmit(handle->huart, tx, sizeof(tx), 10) != HAL_OK) {
    RS485_SetRx(handle);
    return false;
  }
  RS485_SetRx(handle);

  if (HAL_UART_Receive(handle->huart, rx, sizeof(rx), 20) != HAL_OK) {
    return false;
  }

  uint16_t rx_crc = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);
  if (RS485_Crc16(rx, 6) != rx_crc) {
    return false;
  }

  for (uint8_t i = 0; i < 6; i++) {
    if (rx[i] != tx[i]) {
      return false;
    }
  }

  return true;
}
