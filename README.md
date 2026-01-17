# STM32 RS485 Speed Feedback PID Example

This example shows how to read speed feedback from a driver over RS485 (Modbus RTU)
via UART5 and use it as the speed source for a PID loop, then write the PID target
speed back to the driver.

## Driver assumptions
- Modbus RTU slave ID 0x01.
- Speed register at address `0x0020` returning signed RPM.
- Response format: `[id][0x03][0x02][hi][lo][crc_lo][crc_hi]`.
- Target speed register: `0x0000` (Write Single Register, function `0x06`).

Adjust the register address, slave ID, and scaling to match your driver.

## Integration points
- `src/rs485_driver.c`: Modbus RTU request/response with CRC.
- `src/speed_source.c`: caches last valid speed from the driver.
- `src/main.c`: replaces encoder feedback with `SpeedSource_GetRpm()` in the PID loop
  and writes the PID output speed back to the driver every 20 ms.

## Hardware connections
- UART5 TX/RX (PC12/PD2) to RS485 transceiver.
- DE pin (driver enable) on GPIOC PIN12 (update to match your board).

## What to change in your project
1. Replace the original encoder speed calculation with `SpeedSource_GetRpm()`.
2. Initialize UART5 and GPIO for RS485 DE control (PC12/PD2 + DE pin).
3. Adjust Modbus register address and scaling (read 0x0020, write 0x0000).
4. Tune PID gains and speed limits for your motor.
