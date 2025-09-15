#include "flight_controller.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "motors.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "wifi_control.h"
#include "config.h"
#include <math.h>

static const char *TAG = "FLIGHT_CTRL";

// Forward declaration
void flight_controller_set_target_height(int height_cm);

// Calibration parameters
#define CALIBRATION_TIME_MS 30000  // 30 seconds
#define CALIBRATION_SAMPLES 1000
static bool calibration_complete = false;
static int64_t calibration_start_time = 0;
static float accel_bias[3] = {0, 0, 0};
static float gyro_noise_threshold = 0.5f;  // degrees/sec
static float accel_noise_threshold = 0.05f; // g

// Data filtering structures
typedef struct {
    float alpha;  // Filter coefficient (0-1)
    float filtered_value;
} low_pass_filter_t;

typedef struct {
    float buffer[10];
    int index;
    float sum;
} moving_average_t;

// Filters for smooth data
static low_pass_filter_t pitch_filter = {0.8, 0};
static low_pass_filter_t roll_filter = {0.8, 0};
static low_pass_filter_t yaw_filter = {0.9, 0};
static low_pass_filter_t altitude_filter = {0.7, 0};
static moving_average_t height_avg = {{0}, 0, 0};

// Gyro integration for height estimation
static float gyro_vertical_velocity = 0;
static float gyro_estimated_height = 0;
static bool gyro_height_initialized = false;

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

// Low-pass filter function
float apply_low_pass_filter(low_pass_filter_t *filter, float new_value) {
    filter->filtered_value = filter->alpha * filter->filtered_value + (1 - filter->alpha) * new_value;
    return filter->filtered_value;
}

// Moving average function
float apply_moving_average(moving_average_t *avg, float new_value) {
    avg->sum -= avg->buffer[avg->index];
    avg->buffer[avg->index] = new_value;
    avg->sum += new_value;
    avg->index = (avg->index + 1) % 10;
    return avg->sum / 10.0f;
}

// Calibration function
void perform_calibration(float ax, float ay, float az, float gx, float gy, float gz) {
    static float accel_sum[3] = {0, 0, 0};
    static float gyro_max[3] = {0, 0, 0};
    static int sample_count = 0;
    
    if (sample_count < CALIBRATION_SAMPLES) {
        // Accumulate accelerometer readings
        accel_sum[0] += ax;
        accel_sum[1] += ay;
        accel_sum[2] += az;
        
        // Track maximum gyro noise
        if (fabsf(gx) > gyro_max[0]) gyro_max[0] = fabsf(gx);
        if (fabsf(gy) > gyro_max[1]) gyro_max[1] = fabsf(gy);
        if (fabsf(gz) > gyro_max[2]) gyro_max[2] = fabsf(gz);
        
        sample_count++;
        
        if (sample_count == CALIBRATION_SAMPLES) {
            // Calculate accelerometer bias
            accel_bias[0] = accel_sum[0] / CALIBRATION_SAMPLES;
            accel_bias[1] = accel_sum[1] / CALIBRATION_SAMPLES;
            accel_bias[2] = (accel_sum[2] / CALIBRATION_SAMPLES) - 1.0f; // Remove gravity
            
            // Set noise thresholds based on observed noise
            gyro_noise_threshold = fmaxf(gyro_max[0], fmaxf(gyro_max[1], gyro_max[2])) * 2.0f;
            accel_noise_threshold = 0.02f; // Fixed threshold for accelerometer
            
            ESP_LOGI(TAG, "Calibration complete!");
            ESP_LOGI(TAG, "Accel bias: X=%.3f Y=%.3f Z=%.3f", accel_bias[0], accel_bias[1], accel_bias[2]);
            ESP_LOGI(TAG, "Gyro noise threshold: %.3f deg/s", gyro_noise_threshold);
        }
    }
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with windup protection
    pid->integral += error * dt;
    pid->integral = fmaxf(-pid->output_limit, fminf(pid->output_limit, pid->integral));
    float i_term = pid->ki * pid->integral;
    
    // Derivative term with filter
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
    ESP_LOGI(TAG, "Starting 30-second calibration period. Keep drone still!");
    
    calibration_start_time = esp_timer_get_time();
    
    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - last_update_time) / 1000000.0f;
        last_update_time = now;
        
        // Get current orientation from MPU6050
        float ax, ay, az, gx, gy, gz;
        mpu6050_get_motion(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Check if we're still calibrating
        if (!calibration_complete) {
            if ((now - calibration_start_time) < (CALIBRATION_TIME_MS * 1000)) {
                perform_calibration(ax, ay, az, gx, gy, gz);
                
                // Show calibration progress every 5 seconds
                static int64_t last_progress = 0;
                if (now - last_progress > 5000000) {
                    int progress = ((now - calibration_start_time) / 1000) / (CALIBRATION_TIME_MS / 100);
                    ESP_LOGI(TAG, "Calibration progress: %d%%", progress);
                    last_progress = now;
                }
                
                // Send zero telemetry during calibration
                wifi_send_telemetry_extended(0, 0, 0, 0, battery_voltage, 0, false);
                vTaskDelay(pdMS_TO_TICKS(1000 / FLIGHT_LOOP_FREQ));
                continue;
            } else {
                calibration_complete = true;
                ESP_LOGI(TAG, "Calibration period complete. Starting normal operation.");
            }
        }
        
        // Apply calibration corrections
        ax -= accel_bias[0];
        ay -= accel_bias[1];
        az -= accel_bias[2];
        
        // Get angles with complementary filter
        float raw_pitch, raw_roll;
        mpu6050_get_angles(&raw_pitch, &raw_roll);
        
        // Apply low-pass filtering for smooth data
        current_pitch = apply_low_pass_filter(&pitch_filter, raw_pitch);
        current_roll = apply_low_pass_filter(&roll_filter, raw_roll);
        
        // Update yaw with filtered gyro integration
        // Apply deadband to gyro to reduce drift
        if (fabsf(gz) < gyro_noise_threshold) {
            gz = 0;
        }
        float yaw_rate = apply_low_pass_filter(&yaw_filter, gz);
        current_yaw += yaw_rate * dt;
        
        // Normalize yaw to -180 to 180
        while (current_yaw > 180) current_yaw -= 360;
        while (current_yaw < -180) current_yaw += 360;
        
        // Update gyro-based height estimation
        // Calculate vertical acceleration (compensated for tilt)
        float vertical_accel = az * cosf(current_pitch * M_PI / 180) * cosf(current_roll * M_PI / 180);
        
        // Apply deadband to reduce drift
        if (fabsf(vertical_accel) < accel_noise_threshold) {
            vertical_accel = 0;
        }
        
        vertical_accel *= 9.81f; // Convert to m/s²
        
        // Only integrate if we're moving significantly
        if (fabsf(vertical_accel) > 0.1f || fabsf(gyro_vertical_velocity) > 0.1f) {
            gyro_vertical_velocity += vertical_accel * dt;
            gyro_estimated_height += gyro_vertical_velocity * dt * 100; // Convert to cm
        } else {
            // Apply decay to velocity when stationary
            gyro_vertical_velocity *= 0.95f;
        }
        
        // Get altitude from ultrasonic sensors with filtering
        float raw_altitude = ultrasonic_get_altitude();
        if (raw_altitude > 0) {
            current_altitude = apply_moving_average(&height_avg, raw_altitude);
            
            // Initialize gyro height to ultrasonic reading on first valid reading
            if (!gyro_height_initialized && calibration_complete) {
                gyro_estimated_height = current_altitude;
                gyro_height_initialized = true;
                gyro_vertical_velocity = 0;
                ESP_LOGI(TAG, "Gyro height initialized to %.1f cm", gyro_estimated_height);
            }
            
            // Fuse with gyro estimate (complementary filter)
            // Use more ultrasonic when stationary, more gyro when moving
            float fusion_factor = 0.02f;
            if (fabsf(gyro_vertical_velocity) > 10.0f) {
                fusion_factor = 0.005f; // Trust gyro more when moving fast
            }
            
            gyro_estimated_height = (1 - fusion_factor) * gyro_estimated_height + fusion_factor * current_altitude;
            
            // Reset velocity if drift is too large
            if (fabsf(gyro_estimated_height - current_altitude) > 50) {
                gyro_vertical_velocity *= 0.5f;
            }
        }
        
        // Get control inputs
        float throttle, yaw_input, pitch_input, roll_input;
        bool arm_request;
        wifi_get_control(&throttle, &yaw_input, &pitch_input, &roll_input, &arm_request);
        
        // Handle arming/disarming
        if (arm_request && !armed && throttle < 0.1 && calibration_complete) {
            motors_arm();
            armed = true;
            // Reset height references when arming
            gyro_estimated_height = current_altitude;
            gyro_vertical_velocity = 0;
            ESP_LOGI(TAG, "Armed - Height reference reset");
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
            float yaw_output = pid_update(&yaw_pid, target_yaw_rate, yaw_rate, dt);
            
            // Altitude hold
            float altitude_adjust = 0;
            if (altitude_hold_enabled && target_altitude > 0) {
                altitude_adjust = pid_update(&altitude_pid, target_altitude, current_altitude, dt);
                throttle += altitude_adjust;
            }
            
            // Mix outputs for motor control
            int base_throttle = MIN_THROTTLE + (int)(throttle * (MAX_THROTTLE - MIN_THROTTLE));
            
            int motor1 = base_throttle + pitch_output - roll_output - yaw_output;
            int motor2 = base_throttle + pitch_output + roll_output + yaw_output;
            int motor3 = base_throttle - pitch_output + roll_output - yaw_output;
            int motor4 = base_throttle - pitch_output - roll_output + yaw_output;
            
            // Apply motor outputs
            motors_set_throttle(1, motor1);
            motors_set_throttle(2, motor2);
            motors_set_throttle(3, motor3);
            motors_set_throttle(4, motor4);
        } else {
            // Disarmed - set all motors to minimum
            motors_set_all(MIN_THROTTLE);
        }
        
        // Send telemetry with all data including battery
        // Only send gyro height if it's been initialized, otherwise send 0
        float telemetry_gyro_height = gyro_height_initialized ? gyro_estimated_height : 0;
        wifi_send_telemetry_extended(current_altitude, current_pitch, current_roll, 
                                    current_yaw, battery_voltage, telemetry_gyro_height,
                                    armed);
        
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
    
    // Battery voltage filter
    low_pass_filter_t battery_filter = {0.95, 12.6f};
    
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
        float raw_voltage = (adc_reading / 4095.0f) * 3.3f * 4.03f;
        
        // Apply filtering for stable reading
        battery_voltage = apply_low_pass_filter(&battery_filter, raw_voltage);
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Update every 500ms
    }
}

void flight_controller_init(void) {
    ESP_LOGI(TAG, "Initializing flight controller");
    
    // Initialize filters with sensible starting values
    pitch_filter.filtered_value = 0;
    roll_filter.filtered_value = 0;
    yaw_filter.filtered_value = 0;
    altitude_filter.filtered_value = 0;
    
    // Reset gyro height estimation
    gyro_vertical_velocity = 0;
    gyro_estimated_height = 0;
    gyro_height_initialized = false;
    
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