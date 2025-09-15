#include "flight_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/adc.h"  // Add this
#include "esp_adc/adc_oneshot.h"  // Add this for newer ADC API
#include "motors.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "wifi_control.h"
#include "config.h"
#include <math.h>

static const char *TAG = "FLIGHT_CTRL";

// Forward declaration - Add this
void flight_controller_set_target_height(int height_cm);

// PID Controllers
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_limit;
} pid_controller_t;

static pid_controller_t pitch_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
static pid_controller_t roll_pid = {2.0, 0.01, 0.5, 0, 0, MAX_ANGLE};
static pid_controller_t yaw_pid = {3.0, 0.02, 0.0, 0, 0, 200};
static pid_controller_t altitude_pid = {1.5, 0.05, 0.3, 0, 0, 0.5};

// Flight state
static float current_pitch = 0, current_roll = 0, current_yaw = 0;
static float target_pitch = 0, target_roll = 0, target_yaw_rate = 0;
static float current_altitude = 0, target_altitude = 0;
static bool altitude_hold_enabled = false;
static bool armed = false;
static int64_t last_update_time = 0;

// Battery monitoring
static float battery_voltage = 12.6f;
static adc_oneshot_unit_handle_t adc1_handle;

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term
    pid->integral += error * dt;
    pid->integral = fmaxf(-pid->output_limit, fminf(pid->output_limit, pid->integral));
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float d_term = 0;
    if (dt > 0) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;
    
    // Calculate output
    float output = p_term + i_term + d_term;
    output = fmaxf(-pid->output_limit, fminf(pid->output_limit, output));
    
    return output;
}

void flight_controller_task(void *pvParameters) {
    ESP_LOGI(TAG, "Flight controller task started");
    
    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_update_time) / 1000000.0f;
        last_update_time = now;
        
        // Get current orientation from MPU6050
        float ax, ay, az, gx, gy, gz;
        mpu6050_get_motion(&ax, &ay, &az, &gx, &gy, &gz);
        mpu6050_get_angles(&current_pitch, &current_roll);
        
        // Update yaw with gyro integration
        current_yaw += gz * dt;
        
        // Get altitude from ultrasonic sensors
        current_altitude = ultrasonic_get_altitude();
        
        // Get control inputs
        float throttle, yaw_input, pitch_input, roll_input;
        bool arm_request;
        wifi_get_control(&throttle, &yaw_input, &pitch_input, &roll_input, &arm_request);
        
        // Handle arming/disarming
        if (arm_request && !armed && throttle < 0.1) {
            motors_arm();
            armed = true;
            ESP_LOGI(TAG, "Armed");
        } else if (!arm_request && armed) {
            motors_disarm();
            armed = false;
            altitude_hold_enabled = false;
            ESP_LOGI(TAG, "Disarmed");
        }
        
        // Safety checks
        if (armed) {
            // Check battery voltage
            if (battery_voltage < BATTERY_CRITICAL_VOLTAGE) {
                ESP_LOGW(TAG, "Critical battery! Auto-landing");
                throttle = fmaxf(0, throttle - 0.01f);
                if (current_altitude < 10) {
                    motors_disarm();
                    armed = false;
                }
            }
            
            // Check connection
            if (!wifi_is_connected()) {
                ESP_LOGW(TAG, "Connection lost! Holding position");
                pitch_input = 0;
                roll_input = 0;
                yaw_input = 0;
            }
            
            // Check extreme angles
            if (fabsf(current_pitch) > 45 || fabsf(current_roll) > 45) {
                ESP_LOGE(TAG, "Extreme angle detected! Emergency stop");
                motors_disarm();
                armed = false;
            }
        }
        
        if (armed) {
            // Set target angles from input
            target_pitch = pitch_input * MAX_ANGLE;
            target_roll = roll_input * MAX_ANGLE;
            target_yaw_rate = yaw_input * 180.0f; // degrees/sec
            
            // Calculate PID outputs
            float pitch_output = pid_update(&pitch_pid, target_pitch, current_pitch, dt);
            float roll_output = pid_update(&roll_pid, target_roll, current_roll, dt);
            float yaw_output = pid_update(&yaw_pid, target_yaw_rate, gz, dt);
            
            // Altitude hold
            float altitude_adjust = 0;
            if (altitude_hold_enabled && target_altitude > 0) {
                altitude_adjust = pid_update(&altitude_pid, target_altitude, current_altitude, dt);
                throttle += altitude_adjust;
            }
            
            // Mix outputs for motor control
            // X configuration mixing
            int base_throttle = MIN_THROTTLE + (int)(throttle * (MAX_THROTTLE - MIN_THROTTLE));
            
            int motor1 = base_throttle + pitch_output - roll_output - yaw_output; // Front Right
            int motor2 = base_throttle + pitch_output + roll_output + yaw_output; // Front Left
            int motor3 = base_throttle - pitch_output + roll_output - yaw_output; // Rear Left
            int motor4 = base_throttle - pitch_output - roll_output + yaw_output; // Rear Right
            
            // Apply motor outputs
                        // Apply motor outputs
            motors_set_throttle(1, motor1);
            motors_set_throttle(2, motor2);
            motors_set_throttle(3, motor3);
            motors_set_throttle(4, motor4);
        } else {
            // Disarmed - set all motors to minimum
            motors_set_all(MIN_THROTTLE);
        }
        
        // Send telemetry
        wifi_send_telemetry(current_altitude, current_pitch, current_roll, current_yaw);
        
        // Update ultrasonic sensors
        ultrasonic_update_all();
        
        // Run at target frequency
        vTaskDelay(pdMS_TO_TICKS(1000 / FLIGHT_LOOP_FREQ));
    }
}

void battery_monitor_task(void *pvParameters) {
    // Initialize ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
    
    while (1) {
        int adc_reading = 0;
        int raw;
        
        // Average multiple readings
        for (int i = 0; i < 64; i++) {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw));
            adc_reading += raw;
        }
        adc_reading /= 64;
        
        // Convert to voltage (assuming voltage divider 10k/3.3k for 3S battery)
        battery_voltage = (adc_reading / 4095.0f) * 3.3f * 4.03f;
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void flight_controller_init(void) {
    ESP_LOGI(TAG, "Initializing flight controller");
    
    // Create flight control task with high priority
    xTaskCreatePinnedToCore(flight_controller_task, "flight_ctrl", 8192, NULL, 20, NULL, 1);
    
    // Create battery monitor task
    xTaskCreate(battery_monitor_task, "battery_mon", 4096, NULL, 5, NULL);
}

void flight_controller_set_target_height(int height_cm) {
    target_altitude = height_cm;
    altitude_hold_enabled = (height_cm > 0);
    ESP_LOGI(TAG, "Target altitude set to %d cm", height_cm);
}