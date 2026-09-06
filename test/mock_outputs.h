#pragma once

#include "core/failloop.h"
#include "driver/servo.h"

extern float pwm_values[MOTOR_PIN_MAX];
extern uint16_t pwm_rate;
extern failloop_t fault;
