#pragma once

#include <stdbool.h>
#include <stdint.h>

void gyro_control_init();
void gyro_filter_update(bool reset);
void gyro_update();

void gyro_calibrate_bias();
void accel_calibrate();
