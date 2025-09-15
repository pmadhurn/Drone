#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

void flight_controller_init(void);
void flight_controller_task(void *pvParameters);
void flight_controller_set_target_height(int height_cm);
void battery_monitor_task(void *pvParameters);

#endif