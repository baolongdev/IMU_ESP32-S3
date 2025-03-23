#include "I2c_interface.h"

void sensor_read_reg_single(uint8_t addr, uint8_t reg, uint8_t *data_out)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1, true);
  if (Wire.available())
    *data_out = Wire.read();
}

void sensor_read_reg_multi(uint8_t addr, uint8_t start_reg, uint8_t *data_out, uint8_t len)
{
  Wire.beginTransmission(addr);
  Wire.write(start_reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, len, true);
  for (int i = 0; i < len; i++)
    if (Wire.available())
      data_out[i] = Wire.read();
}

void sensor_write_reg_single(uint8_t addr, uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);
}

void board_send_cmd(inter_microcontroller_cmd_t cmd, uint8_t addr)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  Wire.endTransmission();
}

void board_send_data(inter_microcontroller_cmd_t cmd, uint8_t addr, uint8_t *data, uint32_t len)
{
  Wire.beginTransmission(addr);
  Wire.write((uint8_t)cmd);
  Wire.write(data, len);
  Wire.endTransmission();
}
