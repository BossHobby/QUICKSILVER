#pragma once

#include <stdint.h>

void motor_init(void);
void motor_set(uint8_t number, float pwm);
