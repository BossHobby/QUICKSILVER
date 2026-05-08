#pragma once

#include "core/target.h"
#include "driver/gpio.h"

#define SERVO_PULSE_MIN 1000
#define SERVO_PULSE_MAX 2000
#define SERVO_PULSE_CENTER 1500

#define SERVO_PWM_HZ_DEFAULT 50

void servo_init();
void servo_set(uint8_t index, float value);
void servo_update();
void servo_stop();

resource_tag_t servo_alloc_func(gpio_pins_t pin);
