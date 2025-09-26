#include <driver/i2c.h>
#include <stdio.h>

#include "bno055.h"

#define BNO055_ADDR     0x28 // default address when ADR pin is low

#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E

#define BNO055_ACCEL_DATA_X_LSB 0x08
#define BNO055_MAG_DATA_X_LSB   0x0E
#define BNO055_GYRO_DATA_X_LSB  0x14

static int _i2c_port = I2C_NUM_0;

esp_err_t bno055_write(uint8_t reg, uint8_t data) {
  uint8_t buf[2] = {reg, data};
  return i2c_master_write_to_device(_i2c_port, BNO055_ADDR, buf, 2, pdMS_TO_TICKS(100));
}

esp_err_t bno055_read_len(uint8_t reg, uint8_t *data, size_t len) {
  return i2c_master_write_read_device(_i2c_port, BNO055_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t bno055_set_mode(uint8_t mode) {
  return bno055_write(BNO055_OPR_MODE, mode);
}

esp_err_t bno055_init(int i2c_port) {
  _i2c_port = i2c_port;
  vTaskDelay(pdMS_TO_TICKS(700)); // recommended startup delay
                                  // Set to CONFIGMODE to change settings
  bno055_set_mode(0x00);
  vTaskDelay(pdMS_TO_TICKS(20));
  // Set to NDOF mode for full sensor fusion
  return bno055_set_mode(0x0C);
}

int16_t convert(int8_t lsb, int8_t msb) {
  return (int16_t)((msb << 8) | (uint8_t)lsb);
}

esp_err_t bno055_read_9dof(bno055_9dof_t *out) {
  uint8_t buf[18]; // 6 accel + 6 mag + 6 gyro
  esp_err_t ret;

  // Read accelerometer
  ret = bno055_read_len(BNO055_ACCEL_DATA_X_LSB, buf, 6);
  if (ret != ESP_OK) return ret;

  out->accel_x = convert(buf[0], buf[1]) / 100.0f; // m/s² per datasheet
  out->accel_y = convert(buf[2], buf[3]) / 100.0f;
  out->accel_z = convert(buf[4], buf[5]) / 100.0f;

  // Read magnetometer
  ret = bno055_read_len(BNO055_MAG_DATA_X_LSB, buf, 6);
  if (ret != ESP_OK) return ret;

  out->mag_x = convert(buf[0], buf[1]) / 16.0f; // uT
  out->mag_y = convert(buf[2], buf[3]) / 16.0f;
  out->mag_z = convert(buf[4], buf[5]) / 16.0f;

  // Read gyroscope
  ret = bno055_read_len(BNO055_GYRO_DATA_X_LSB, buf, 6);
  if (ret != ESP_OK) return ret;

  out->gyro_x = convert(buf[0], buf[1]) / 16.0f; // deg/s
  out->gyro_y = convert(buf[2], buf[3]) / 16.0f;
  out->gyro_z = convert(buf[4], buf[5]) / 16.0f;

  return ESP_OK;
}

