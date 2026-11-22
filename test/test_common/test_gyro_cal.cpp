#include <unity.h>
#include <math.h>

#include "control/gyro_cal.h"

#include "util/util.h"

static constexpr float SAMPLE_TIME = 0.002f; // seconds
static constexpr float ACC_1G_RAW = 2048.0f;

static uint32_t noise_state;

// Uniform noise in [-amplitude, amplitude], the same sequence every test.
static float noise(float amplitude) {
  noise_state = noise_state * 1664525u + 1013904223u;
  return amplitude * ((float)(noise_state >> 8) / (float)(1u << 23) - 1.0f);
}

static vec3_t noisy(vec3_t value, float amplitude) {
  return {{value.axis[0] + noise(amplitude), value.axis[1] + noise(amplitude), value.axis[2] + noise(amplitude)}};
}

// Gravity as the accelerometer reads it after rolling by angle.
static vec3_t gravity_rolled(float angle) {
  return {{0.0f, ACC_1G_RAW * sinf(angle), ACC_1G_RAW * cosf(angle)}};
}

void test_gyro_cal_resting_window_averages_offset(void) {
  noise_state = 1;
  const vec3_t offset = {{1.5f, -0.8f, 0.3f}};

  gyro_cal_window_t window;
  gyro_cal_reset(&window);
  for (uint32_t i = 0; i < GYRO_CAL_SAMPLES - 1; i++) {
    TEST_ASSERT_EQUAL(GYRO_CAL_COLLECTING, gyro_cal_push(&window, noisy(offset, 0.15f), noisy(gravity_rolled(0.0f), 8.0f)));
  }
  TEST_ASSERT_EQUAL(GYRO_CAL_PASSED, gyro_cal_push(&window, noisy(offset, 0.15f), noisy(gravity_rolled(0.0f), 8.0f)));
  TEST_ASSERT_FLOAT_WITHIN(0.02f, offset.roll, window.zero.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, offset.pitch, window.zero.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, offset.yaw, window.zero.yaw);
}

void test_gyro_cal_changing_rate_restarts_window(void) {
  noise_state = 1;

  gyro_cal_window_t window;
  gyro_cal_reset(&window);

  // A yaw rate rising by 1 deg/s per window leaves the accelerometer alone.
  bool rejected = false;
  for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++) {
    const vec3_t gyro = {{0.0f, 0.0f, (float)i / (float)GYRO_CAL_SAMPLES}};
    const gyro_cal_result_t result = gyro_cal_push(&window, noisy(gyro, 0.15f), noisy(gravity_rolled(0.0f), 8.0f));
    TEST_ASSERT_NOT_EQUAL(GYRO_CAL_PASSED, result);
    rejected = rejected || result == GYRO_CAL_REJECTED;
  }
  TEST_ASSERT_TRUE(rejected);

  // At rest again, the next whole window passes without the earlier samples.
  gyro_cal_reset(&window);
  const vec3_t offset = {{0.0f, 0.0f, 1.0f}};
  gyro_cal_result_t result = GYRO_CAL_COLLECTING;
  for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++) {
    result = gyro_cal_push(&window, noisy(offset, 0.15f), noisy(gravity_rolled(0.0f), 8.0f));
  }
  TEST_ASSERT_EQUAL(GYRO_CAL_PASSED, result);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, window.zero.yaw);
}

void test_gyro_cal_steady_roll_never_passes(void) {
  noise_state = 1;

  gyro_cal_window_t window;
  gyro_cal_reset(&window);

  // A steady 2 deg/s roll is a constant gyro reading; only gravity turns.
  const vec3_t gyro = {{2.0f, 0.0f, 0.0f}};
  bool rejected = false;
  for (uint32_t i = 0; i < 4 * GYRO_CAL_SAMPLES; i++) {
    const float angle = 2.0f * DEGTORAD * SAMPLE_TIME * (float)i;
    const gyro_cal_result_t result = gyro_cal_push(&window, noisy(gyro, 0.15f), noisy(gravity_rolled(angle), 8.0f));
    TEST_ASSERT_NOT_EQUAL(GYRO_CAL_PASSED, result);
    rejected = rejected || result == GYRO_CAL_REJECTED;
  }
  TEST_ASSERT_TRUE(rejected);
}
