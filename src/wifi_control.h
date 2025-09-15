#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include <stdbool.h>

void wifi_control_init(void);
void control_receiver_task(void *pvParameters);
void wifi_send_telemetry(float height, float pitch, float roll, float yaw);
void wifi_send_telemetry_extended(float height, float pitch, float roll, float yaw, 
                                  float battery, float gyro_height, bool armed);
void wifi_get_control(float *throttle, float *yaw, float *pitch, float *roll, bool *armed);
bool wifi_is_connected(void);

#endif // WIFI_CONTROL_H