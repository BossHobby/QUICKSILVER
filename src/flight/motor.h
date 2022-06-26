#pragma once

#include <stdbool.h>

void motor_test_calc(bool motortest_usb, float mix[4]);
void motor_mixer_calc(float mix[4]);

void motor_output_calc(float mix[4]);