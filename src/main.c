#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_control.h"
#include "flight_controller.h"
#include "motors.h"
#include "mpu6050.h"
#include "ultrasonic.h"

static const char *TAG = "DRONE_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "Drone Controller Starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize components
    motors_init();
    mpu6050_init();
    ultrasonic_init();
    wifi_control_init();
    flight_controller_init();
    
    ESP_LOGI(TAG, "All systems initialized. Ready for flight!");
    
    // Main loop runs in flight controller task
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}