#pragma once

#include "core/target.h"

#define SERVO_PULSE_MIN 1000
#define SERVO_PULSE_MAX 2000
#define SERVO_PULSE_CENTER 1500

#define SERVO_MAX 4
#define SERVO_PWM_HZ_DEFAULT 50

typedef enum {
  SERVO_STEERING = 0,
  SERVO_DRIVE = 1,
} servo_channel_t;

void servo_init();
void servo_set(servo_channel_t index, float value);
void servo_write();
void servo_stop();
void servo_pwm_init(const gpio_pins_t *pins, uint16_t pwm_hz);
void servo_pwm_write(const float *values);
void servo_pwm_stop();
