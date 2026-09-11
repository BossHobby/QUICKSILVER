#pragma once

#define ACC_1G 1.0f

void imu_init();
void imu_filter_update();
// Update gravity, attitude, and heading from the current sensor sample.
void imu_update();
