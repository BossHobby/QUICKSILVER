#pragma once

#define ACC_1G 1.0f

void imu_init();
void imu_filter_update();
void imu_calc();
// Navigation saw the vehicle move against its commands: relearn the heading
// from GPS course before trusting it again.
void imu_reject_heading();
// Navigation measured the heading error in radians, true minus estimated,
// from how GPS velocity changed against the accelerometer over dt seconds.
void imu_observe_heading(float error, float dt);
