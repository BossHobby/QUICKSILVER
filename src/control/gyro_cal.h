#pragma once

#include <stdint.h>

#include "util/vector.h"

// A gyro calibration window is GYRO_CAL_BLOCKS consecutive block averages the
// vehicle rested through. Averaging a block takes the sensor noise out of the
// comparison, so the limits bound the error of the offset itself.
static constexpr uint32_t GYRO_CAL_BLOCKS = 8;
static constexpr uint32_t GYRO_CAL_BLOCK_SAMPLES = 64;
static constexpr uint32_t GYRO_CAL_SAMPLES = GYRO_CAL_BLOCKS * GYRO_CAL_BLOCK_SAMPLES;

typedef enum {
  GYRO_CAL_COLLECTING,
  GYRO_CAL_REJECTED, // the vehicle moved: the window started over
  GYRO_CAL_PASSED,   // zero holds the average of the window
} gyro_cal_result_t;

typedef struct {
  uint32_t samples; // pushed into this window
  vec3_t gyro_sum;  // of the block being filled
  vec3_t accel_sum;
  vec3_t gyro_min; // over the finished block averages
  vec3_t gyro_max;
  vec3_t gyro_total;  // of the finished block averages
  vec3_t accel_first; // sum of the first block, the gravity reference
  vec3_t zero;        // deg/s, valid after GYRO_CAL_PASSED
} gyro_cal_window_t;

void gyro_cal_reset(gyro_cal_window_t *window);
// Takes one sample: gyro in deg/s, accel in any unit.
gyro_cal_result_t gyro_cal_push(gyro_cal_window_t *window, vec3_t gyro, vec3_t accel);
