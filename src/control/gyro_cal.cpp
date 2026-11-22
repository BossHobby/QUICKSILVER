#include "control/gyro_cal.h"

#include <math.h>

#include "util/util.h"

// A changing rate is handling: block averages further apart than this on any
// axis reject the window.
static constexpr float MAX_RATE_SPREAD = 0.2f; // deg/s
// A steady turn reads as a constant rate, like an offset. The accelerometer
// shows it unless it turns about gravity: a block whose gravity points further
// than this from the first block's rejects the window, bounding the roll and
// pitch error to about 0.5 deg/s.
static constexpr float MAX_TILT = 0.5f * DEGTORAD;

// The cross product's length is the sine of the angle, here the angle itself.
static bool gyro_cal_tilted(vec3_t from, vec3_t to) {
  const vec3_t cross = vec3_cross(from, to);
  return vec3_dot(cross, cross) > MAX_TILT * MAX_TILT * vec3_dot(from, from) * vec3_dot(to, to);
}

void gyro_cal_reset(gyro_cal_window_t *window) {
  *window = {};
}

gyro_cal_result_t gyro_cal_push(gyro_cal_window_t *window, vec3_t gyro, vec3_t accel) {
  window->gyro_sum += gyro;
  window->accel_sum += accel;
  if (++window->samples % GYRO_CAL_BLOCK_SAMPLES != 0)
    return GYRO_CAL_COLLECTING;

  const vec3_t rate = window->gyro_sum * (1.0f / GYRO_CAL_BLOCK_SAMPLES);
  const vec3_t gravity = window->accel_sum;
  window->gyro_sum = {};
  window->accel_sum = {};

  if (window->samples == GYRO_CAL_BLOCK_SAMPLES) {
    window->gyro_min = rate;
    window->gyro_max = rate;
    window->accel_first = gravity;
  }
  bool moved = gyro_cal_tilted(window->accel_first, gravity);
  for (uint8_t i = 0; i < 3; i++) {
    window->gyro_min.axis[i] = fminf(window->gyro_min.axis[i], rate.axis[i]);
    window->gyro_max.axis[i] = fmaxf(window->gyro_max.axis[i], rate.axis[i]);
    if (window->gyro_max.axis[i] - window->gyro_min.axis[i] > MAX_RATE_SPREAD)
      moved = true;
  }
  if (moved) {
    gyro_cal_reset(window);
    return GYRO_CAL_REJECTED;
  }

  window->gyro_total += rate;
  if (window->samples < GYRO_CAL_SAMPLES)
    return GYRO_CAL_COLLECTING;

  window->zero = window->gyro_total * (1.0f / GYRO_CAL_BLOCKS);
  return GYRO_CAL_PASSED;
}
