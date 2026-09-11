#pragma once

#include <stdbool.h>
#include <stdint.h>

void sixaxis_init();
// Rebuild the sensor rotation after changing profile.motor.gyro_orientation.
void sixaxis_orientation_update();
void sixaxis_read();

void sixaxis_gyro_cal();
void sixaxis_acc_cal();
