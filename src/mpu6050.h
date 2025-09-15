#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

void mpu6050_init(void);
void mpu6050_read_byte(uint8_t reg, uint8_t *data);
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
void mpu6050_get_motion(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void mpu6050_calibrate_gyro(void);
void mpu6050_get_angles(float *pitch, float *roll);

#endif