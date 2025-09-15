#include "motors.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "config.h"

static const char *TAG = "MOTORS";

static const int motor_pins[4] = {MOTOR_1_PIN, MOTOR_2_PIN, MOTOR_3_PIN, MOTOR_4_PIN};
static bool motors_armed = false;

void motors_init(void) {
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure LEDC channels for each motor
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = i,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = motor_pins[i],
            .duty           = 0,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
    
    // Initialize all motors to minimum throttle
    motors_set_all(MIN_THROTTLE);
    ESP_LOGI(TAG, "Motors initialized");
}

void motors_arm(void) {
    if (!motors_armed) {
        motors_armed = true;
        motors_set_all(ARM_THROTTLE);
        ESP_LOGI(TAG, "Motors ARMED");
    }
}

void motors_disarm(void) {
    motors_armed = false;
    motors_set_all(MIN_THROTTLE);
    ESP_LOGI(TAG, "Motors DISARMED");
}

void motors_set_throttle(int motor, int throttle) {
    if (motor < 1 || motor > 4) return;
    
    // Constrain throttle
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;
    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;
    
    // Only allow throttle above minimum if armed
    if (!motors_armed && throttle > MIN_THROTTLE) {
        throttle = MIN_THROTTLE;
    }
    
    // Convert microseconds to duty cycle (50Hz = 20ms period)
    // 8192 = 2^13 (13-bit resolution)
    uint32_t duty = (throttle * 8192) / 20000;
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, motor - 1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, motor - 1));
}

void motors_set_all(int throttle) {
    for (int i = 1; i <= 4; i++) {
        motors_set_throttle(i, throttle);
    }
}

bool motors_is_armed(void) {
    return motors_armed;
}
