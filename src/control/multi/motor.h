#pragma once

#include <stdbool.h>

#include "core/project.h"

#define MULTI_MOTOR_COUNT 4

void motor_test_calc(bool motortest_usb, float mix[MULTI_MOTOR_COUNT]);
void motor_mixer_calc(float mix[MULTI_MOTOR_COUNT]);

void motor_output_calc(float mix[MULTI_MOTOR_COUNT]);
