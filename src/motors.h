#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>

void motors_init(void);
void motors_arm(void);
void motors_disarm(void);
void motors_set_throttle(int motor, int throttle);
void motors_set_all(int throttle);
bool motors_is_armed(void);

#endif
