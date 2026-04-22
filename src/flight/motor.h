#pragma once

#include <stdbool.h>

#include "core/project.h"

void motor_test_calc(bool motortest_usb, float mix[MOTOR_PIN_MAX]);
void motor_mixer_calc(float mix[MOTOR_PIN_MAX]);

void motor_output_calc(float mix[MOTOR_PIN_MAX]);

#ifdef VEHICLE_ROVER
void rover_pid_init();
void rover_mixer_calc(float *motor_out, float *servo_out);
void rover_motor_output_calc(float motor_out, float servo_out);
void rover_test_calc(bool motortest_usb, float *motor_out, float *servo_out);
void rover_heading_reset();
#endif
