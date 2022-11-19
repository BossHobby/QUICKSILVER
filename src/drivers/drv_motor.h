#pragma once

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_OFF -1.0f

typedef enum {
  MOTOR_FORWARD,
  MOTOR_REVERSE
} motor_direction_t;

#define MOTOR_BL 0
#define MOTOR_FL 1
#define MOTOR_BR 2
#define MOTOR_FR 3

// driver functions
void motor_init();
void motor_wait_for_ready();
void motor_beep();
void motor_write(float *values);
void motor_set_direction(motor_direction_t dir);
bool motor_direction_change_done();

// generic functions
void motor_set(uint8_t number, float pwm);
void motor_set_all(float pwm);
void motor_update();