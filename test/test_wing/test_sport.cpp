#include <unity.h>

#include <math.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/pid.h"
#include "core/profile.h"

static void arm_sport() {
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | (1U << AUX_SPORTMODE);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 0.5f;
  control();
  TEST_ASSERT_TRUE(flags.in_air);
}

static void gains(float p, float i, float d) {
  auto *rates = profile_current_pid_rates();
  rates->kp = {{p, p, p}};
  rates->ki = {{i, i, i}};
  rates->kd = {{d, d, d}};
  pid_rates_update();
}

void test_sport_passes_sticks_and_damps_rotation() {
  gains(62.8f, 0.0f, 0.0f);
  arm_sport();
  state.rx_filtered.roll = 0.4f;
  control();
  TEST_ASSERT_EQUAL_STRING("SPORT", control_flight_mode_name());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.setpoint.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, state.mixer_source[OUTPUT_SOURCE_ROLL]);
  state.gyro.roll = 0.5f;
  control();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.05f, state.pid_p_term.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.35f, state.mixer_source[OUTPUT_SOURCE_ROLL]);
  state.aux_active |= 1U << AUX_ACROMODE;
  control();
  TEST_ASSERT_EQUAL_STRING("ACRO", control_flight_mode_name());
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
}

void test_sport_hold_releases_on_input_and_disarm() {
  gains(0.0f, 100.0f, 0.0f);
  arm_sport();
  state.gyro.roll = 0.1f;
  for (unsigned i = 0; i < 200; i++)
    control();
  const float hold = state.pid_i_term.roll;
  TEST_ASSERT_TRUE(hold < -0.01f);
  state.gyro.roll = 0.0f;
  state.rx_filtered.roll = 0.5f;
  for (unsigned i = 0; i < 50; i++)
    control();
  TEST_ASSERT_TRUE(fabsf(state.pid_i_term.roll) < fabsf(hold) * 0.5f);
  for (unsigned i = 0; i < 300; i++)
    control();
  const float frozen = state.pid_i_term.roll;
  state.gyro.roll = 0.5f;
  for (unsigned i = 0; i < 100; i++)
    control();
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, frozen, state.pid_i_term.roll);
  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_i_term.roll);
}

void test_sport_reentry_discards_manual_gyro_gap() {
  gains(0.0f, 0.0f, 100.0f);
  arm_sport();
  state.aux_active &= ~(1U << AUX_SPORTMODE);
  control();
  state.gyro.roll = 0.5f;
  state.aux_active |= 1U << AUX_SPORTMODE;
  control();
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0f, state.pid_d_term.roll);
}
