#ifndef __BNO055_H__
#define __BNO055_H__
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus

#include <freertos/FreeRTOS.h>

typedef struct {
  float accel_x, accel_y, accel_z; // m/s²
  float gyro_x, gyro_y, gyro_z;    // deg/s
  float mag_x, mag_y, mag_z;       // uT
} bno055_9dof_t;

esp_err_t bno055_init(int i2c_port);
esp_err_t bno055_read_9dof(bno055_9dof_t *out);

#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__BNO055_H__

//#define BNO055_IMPLEMENTATIONS
#ifdef BNO055_IMPLEMENTATIONS
#ifndef __BNO055_IMP__
#define __BNO055_IMP__
#ifdef __cplusplus
extern "C" {
#endif//__cplusplus



#ifdef __cplusplus
}
#endif//__cplusplus
#endif//__BNO055_IMP__
#undef BNO055_IMPLEMENTATIONS
#endif//BNO055_IMPLEMENTATIONS

