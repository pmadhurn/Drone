#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "config.h"
#include <math.h>

static const char *TAG = "MPU6050";

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_WHO_AM_I 0x75

static float gyro_offset[3] = {0};
static float accel_scale = 16384.0f;  // ±2g
static float gyro_scale = 131.0f;     // ±250°/s

void mpu6050_init(void) {
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    
    // Wake up MPU6050
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);
    
    // Verify connection
    uint8_t who_am_i;
    mpu6050_read_byte(MPU6050_WHO_AM_I, &who_am_i);
    ESP_LOGI(TAG, "MPU6050 WHO_AM_I: 0x%02X", who_am_i);
    
    // Calibrate gyroscope
    mpu6050_calibrate_gyro();
}void mpu6050_read_byte(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t data[14];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    // Combine high and low bytes
    accel[0] = (data[0] << 8) | data[1];
    accel[1] = (data[2] << 8) | data[3];
    accel[2] = (data[4] << 8) | data[5];
    
    gyro[0] = (data[8] << 8) | data[9];
    gyro[1] = (data[10] << 8) | data[11];
    gyro[2] = (data[12] << 8) | data[13];
}

void mpu6050_get_motion(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    int16_t accel[3], gyro[3];
    mpu6050_read_raw(accel, gyro);
    
    // Convert to real units
    *ax = accel[0] / accel_scale;
    *ay = accel[1] / accel_scale;
    *az = accel[2] / accel_scale;
    
    *gx = (gyro[0] / gyro_scale) - gyro_offset[0];
    *gy = (gyro[1] / gyro_scale) - gyro_offset[1];
    *gz = (gyro[2] / gyro_scale) - gyro_offset[2];
}

void mpu6050_calibrate_gyro(void) {
    ESP_LOGI(TAG, "Calibrating gyroscope... Keep drone still!");
    
    float sum[3] = {0};
    int samples = 1000;
    
    for (int i = 0; i < samples; i++) {
        int16_t accel[3], gyro[3];
        mpu6050_read_raw(accel, gyro);
        
        sum[0] += gyro[0] / gyro_scale;
        sum[1] += gyro[1] / gyro_scale;
        sum[2] += gyro[2] / gyro_scale;
        
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    
    gyro_offset[0] = sum[0] / samples;
    gyro_offset[1] = sum[1] / samples;
    gyro_offset[2] = sum[2] / samples;
    
    ESP_LOGI(TAG, "Gyro calibration complete. Offsets: X=%.2f Y=%.2f Z=%.2f",
             gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

void mpu6050_get_angles(float *pitch, float *roll) {
    float ax, ay, az, gx, gy, gz;
    mpu6050_get_motion(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Calculate angles from accelerometer
    *pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    *roll = atan2f(ay, az) * 180.0f / M_PI;
}