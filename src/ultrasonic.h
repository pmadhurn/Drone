#ifndef ULTRASONIC_H
#define ULTRASONIC_H

void ultrasonic_init(void);
void ultrasonic_trigger(int sensor);
float ultrasonic_get_distance(int sensor);
float ultrasonic_get_altitude(void);
void ultrasonic_update_all(void);

#endif