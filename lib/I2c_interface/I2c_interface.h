#ifndef ARDUINO_I2C_INTERFACE_H
#define ARDUINO_I2C_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>

enum inter_microcontroller_cmd_t{
  MASTER_WHEEL_CONTROL_DRIVE, 
  MASTER_WHEEL_CONTROL_STOP, 
  NOP,
};

void sensor_read_reg_single(uint8_t addr, uint8_t reg, uint8_t *data_out);
void sensor_read_reg_multi(uint8_t addr, uint8_t start_reg, uint8_t *data_out, uint8_t len);
void sensor_write_reg_single(uint8_t addr, uint8_t reg, uint8_t data);

void board_send_cmd(inter_microcontroller_cmd_t cmd, uint8_t addr);
void board_send_data(inter_microcontroller_cmd_t cmd, uint8_t addr, uint8_t *data, uint32_t len);

#endif
