#pragma once

#include <stdint.h>

#define FORWARD 0
#define REVERSE 1

#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_BR 2
#define MOTOR_FR 3

void motor_init();
void motor_set(uint8_t number, float pwm);
void motor_wait_for_ready();
void motor_set_all(float pwm);
void motor_beep();