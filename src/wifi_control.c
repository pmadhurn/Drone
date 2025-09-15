#include "wifi_control.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"  // Add this for MAC address macros
#include "lwip/sockets.h"
#include "config.h"
#include "flight_controller.h"  // Add this
#include <string.h>

static const char *TAG = "WIFI_CONTROL";

static int control_socket = -1;
static int telemetry_socket = -1;
static struct sockaddr_in client_addr;
static bool client_connected = false;

typedef struct {
    float throttle;
    float yaw;
    float pitch;
    float roll;
    int armed;
} control_packet_t;

static control_packet_t last_control = {0};

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Station disconnected: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
        client_connected = false;
    }
}

void control_receiver_task(void *pvParameters) {
    uint8_t rx_buffer[128];
    
    while (1) {
        socklen_t addr_len = sizeof(client_addr);
        int len = recvfrom(control_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,
                          (struct sockaddr *)&client_addr, &addr_len);
        
        if (len > 0) {
            client_connected = true;
            
            // Parse control packet
            if (len >= sizeof(control_packet_t)) {
                memcpy(&last_control, rx_buffer, sizeof(control_packet_t));
            } else if (len >= 8 && *(uint32_t*)rx_buffer == 0xFF) {
                // Height command
                int target_height = *(uint32_t*)(rx_buffer + 4);
                flight_controller_set_target_height(target_height);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void wifi_control_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASS,
            .max_connection = 1,
            .authmode = WIFI_AUTH_WPA2_PSK
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started. SSID: %s", WIFI_SSID);
    
    // Create control socket
    control_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONTROL_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY)
    };
    bind(control_socket, (struct sockaddr *)&server_addr, sizeof(server_addr));
    
    // Create telemetry socket
    telemetry_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    
    // Start control receiver task
    xTaskCreate(control_receiver_task, "control_rx", 4096, NULL, 10, NULL);
}

void wifi_send_telemetry(float height, float pitch, float roll, float yaw) {
    if (!client_connected) return;
    
    float telemetry[4] = {height, pitch, roll, yaw};
    
    client_addr.sin_port = htons(TELEMETRY_PORT);
    sendto(telemetry_socket, telemetry, sizeof(telemetry), 0,
           (struct sockaddr *)&client_addr, sizeof(client_addr));
}

void wifi_get_control(float *throttle, float *yaw, float *pitch, float *roll, bool *armed) {
    *throttle = last_control.throttle;
    *yaw = last_control.yaw;
    *pitch = last_control.pitch;
    *roll = last_control.roll;
    *armed = last_control.armed;
}

bool wifi_is_connected(void) {
    return client_connected;
}