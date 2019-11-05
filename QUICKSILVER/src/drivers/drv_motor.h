#pragma once

#include <stdint.h>

#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_BR 2
#define MOTOR_FR 3

void motor_init(void);
void motor_set(uint8_t number, float pwm);
void motor_set_all(float pwm);