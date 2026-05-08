#pragma once

#include <stdbool.h>
#include <float.h>
#include <stdint.h>

#define MOTOR_OFF (-FLT_MAX)

typedef enum {
  MOTOR_FORWARD,
  MOTOR_REVERSE
} motor_direction_t;

typedef enum {
  MOTOR_BL,
  MOTOR_FL,
  MOTOR_BR,
  MOTOR_FR,
} motor_position_t;

// driver functions
void motor_init();
void motor_wait_for_ready();
void motor_beep();
void motor_set_direction(motor_direction_t dir);
bool motor_direction_change_done();

// generic functions
void motor_set(uint8_t pos, float pwm);
void motor_set_all(float pwm);
void motor_update();
