#include "ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"  // Add this
#include "esp_rom_sys.h"  // Add this for delay
#include "esp_log.h"
#include "config.h"
#include <sys/time.h>

static const char *TAG = "ULTRASONIC";

typedef struct {
    int trig_pin;
    int echo_pin;
    int64_t start_time;
    float distance;
} ultrasonic_sensor_t;

static ultrasonic_sensor_t sensors[4] = {
    {US1_TRIG_PIN, US1_ECHO_PIN, 0, 0},
    {US2_TRIG_PIN, US2_ECHO_PIN, 0, 0},
    {US3_TRIG_PIN, US3_ECHO_PIN, 0, 0},
    {US4_TRIG_PIN, US4_ECHO_PIN, 0, 0}
};

static void IRAM_ATTR echo_isr_handler(void* arg) {
    int sensor_idx = (int)arg;
    if (gpio_get_level(sensors[sensor_idx].echo_pin)) {
        sensors[sensor_idx].start_time = esp_timer_get_time();
    } else {
        int64_t duration = esp_timer_get_time() - sensors[sensor_idx].start_time;
        sensors[sensor_idx].distance = (duration * 0.034) / 2.0; // cm
    }
}

void ultrasonic_init(void) {
    for (int i = 0; i < 4; i++) {
        // Configure trigger pins
        gpio_reset_pin(sensors[i].trig_pin);  // Use gpio_reset_pin instead
        gpio_set_direction(sensors[i].trig_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(sensors[i].trig_pin, 0);
        
        // Configure echo pins
        gpio_reset_pin(sensors[i].echo_pin);  // Use gpio_reset_pin instead
        gpio_set_direction(sensors[i].echo_pin, GPIO_MODE_INPUT);
        gpio_set_intr_type(sensors[i].echo_pin, GPIO_INTR_ANYEDGE);
    }
    
    // Install ISR service once
    gpio_install_isr_service(0);
    
    // Add ISR handlers
    for (int i = 0; i < 4; i++) {
        gpio_isr_handler_add(sensors[i].echo_pin, echo_isr_handler, (void*)i);
    }
    
    ESP_LOGI(TAG, "Ultrasonic sensors initialized");
}

void ultrasonic_trigger(int sensor) {
    if (sensor < 0 || sensor > 3) return;
    
    gpio_set_level(sensors[sensor].trig_pin, 1);
    esp_rom_delay_us(10);  // Use esp_rom_delay_us instead
    gpio_set_level(sensors[sensor].trig_pin, 0);
}

float ultrasonic_get_distance(int sensor) {
    if (sensor < 0 || sensor > 3) return 0;
    return sensors[sensor].distance;
}

float ultrasonic_get_altitude(void) {
    // Average all 4 sensors at 45° for altitude estimation
    float sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 4; i++) {
        float dist = sensors[i].distance;
        if (dist > 0 && dist < 400) { // Valid range 0-400cm
            // Correct for 45° angle (multiply by cos(45°) ≈ 0.707)
            sum += dist * 0.707;
            valid_readings++;
        }
    }
    
    return (valid_readings > 0) ? (sum / valid_readings) : 0;
}

void ultrasonic_update_all(void) {
    static int current_sensor = 0;
    
    // Trigger one sensor at a time to avoid interference
    ultrasonic_trigger(current_sensor);
    current_sensor = (current_sensor + 1) % 4;
}