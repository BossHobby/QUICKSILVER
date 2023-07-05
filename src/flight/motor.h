#pragma once

#include <stdbool.h>

#include "core/project.h"

void motor_test_calc(bool motortest_usb, float mix[MOTOR_PIN_MAX]);
void motor_mixer_calc(float mix[MOTOR_PIN_MAX]);

void motor_output_calc(float mix[MOTOR_PIN_MAX]);