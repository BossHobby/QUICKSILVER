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
// Persist the ESC direction for a logical profile output while disarmed and not testing.
// Success means the command sequence was transmitted, not acknowledged by the ESC.
bool motor_configure_direction(uint8_t index, motor_direction_t dir);
void motor_set_direction(motor_direction_t dir);
bool motor_direction_change_done();

// generic functions
void motor_set(uint8_t pos, float pwm);
void motor_set_all(float pwm);
void motor_update();
