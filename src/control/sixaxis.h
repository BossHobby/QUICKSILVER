#pragma once

#include <stdbool.h>
#include <stdint.h>

void sixaxis_init();
void sixaxis_filter_update(bool reset);
// Rebuild the sensor rotation after changing profile.motor.gyro_orientation.
void sixaxis_orientation_update();
void sixaxis_read();

// Boot calibration: a resting window becomes the gyro offset unless it
// contradicts the stored one, which otherwise stands in.
void sixaxis_gyro_boot_cal();
// Commanded calibration: a resting window becomes the gyro offset and the
// stored one, which the caller's flash_save() persists.
void sixaxis_gyro_cal();
void sixaxis_acc_cal();
