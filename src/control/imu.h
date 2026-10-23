#pragma once

#define ACC_1G 1.0f

void imu_init();
void imu_filter_update();
void imu_calc();
// Navigation saw the vehicle move against its commands: relearn the heading
// from GPS course before trusting it again.
void imu_reject_heading();
