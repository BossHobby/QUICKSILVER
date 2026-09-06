#include <bit>
#include <math.h>
#include <unity.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/pid.h"
#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "mock_outputs.h"
#include "util/util.h"

// Script sensor inputs and advance the real controller, as in navigation tests.
// This checks command direction and transitions, not airframe dynamics.
static bool finite_float(float value) {
  // Firmware flags include fast-math, which can optimize isfinite() to true.
  return (std::bit_cast<uint32_t>(value) & 0x7f800000U) != 0x7f800000U;
}

static void tick(uint32_t milliseconds = 1) {
  for (uint32_t i = 0; i < milliseconds; i++) {
    time_test_advance_us(1000);
    control();
    for (uint8_t axis = 0; axis < 3; axis++) {
      TEST_ASSERT_TRUE(finite_float(state.setpoint.axis[axis]));
      TEST_ASSERT_TRUE(finite_float(state.pidoutput.axis[axis]));
    }
    for (uint8_t output = 1; output < 4; output++) {
      TEST_ASSERT_TRUE(finite_float(pwm_values[output]));
      TEST_ASSERT_TRUE(fabsf(pwm_values[output]) <= 1.0f);
    }
  }
}

static void prepare(uint32_t mode) {
  profile_set_defaults(); // Include the actual default gains, not the zero-gain template.
  profile.receiver.aux[AUX_PREARM].channel = RX_CHANNEL_ON;
  state.looptime_autodetect = 1000;
  state.rx_filter_hz = 50;
  state.vbat_cell_avg = 4.0f;
  pid_init();
  tick(); // Reset private launch/trim state with switches off.
  state.aux_active = (1U << AUX_PREARM) | mode;
  tick();
  state.aux_active |= 1U << AUX_ARMING;
  tick();
  TEST_ASSERT_TRUE(flags.arm_state);
}

static void test_wing_angle_corrects_roll_and_pitch_in_both_directions() {
  prepare(1U << AUX_LEVELMODE);
  const float angles[] = {-80, -45, -10, 10, 45, 80};
  for (uint8_t axis = 0; axis < 2; axis++) {
    for (float angle : angles) {
      pid_reset_i();
      state.GEstG = {};
      state.GEstG.axis[axis] = sinf(angle * DEGTORAD);
      state.GEstG.yaw = cosf(angle * DEGTORAD);
      tick(20);
      TEST_ASSERT_TRUE(state.setpoint.axis[axis] * angle < 0);
      TEST_ASSERT_TRUE(state.pidoutput.axis[axis] * angle < 0);
      // Default elevons: roll is differential; pitch is common direction.
      const float applied = axis == 0 ? pwm_values[1] - pwm_values[2] : pwm_values[1] + pwm_values[2];
      TEST_ASSERT_TRUE(applied * angle < 0);
    }
  }
}

static void test_wing_angle_stick_commands_and_manual_handoff() {
  prepare(1U << AUX_LEVELMODE);
  state.rx_filtered.roll = 0.25f;
  state.rx_filtered.pitch = -0.15f;
  tick(100);
  TEST_ASSERT_TRUE(state.setpoint.roll > 0);
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0);
  state.aux_active &= ~(1U << AUX_LEVELMODE);
  tick();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.10f, pwm_values[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.40f, pwm_values[2]);
  TEST_ASSERT_EQUAL_FLOAT(0, pid_get_ierror()->roll);
  TEST_ASSERT_EQUAL_FLOAT(0, pid_get_ierror()->pitch);
  state.rx_filtered = {};
  state.aux_active |= 1U << AUX_ACROMODE;
  tick(100);
  TEST_ASSERT_EQUAL_FLOAT(0, state.setpoint.roll);
  TEST_ASSERT_EQUAL_FLOAT(0, state.setpoint.pitch);
}

static void prepare_launch() {
  prepare(1U << AUX_AUTOLAUNCH);
  profile.wing.autolaunch.idle_throttle = 0;
  profile.wing.autolaunch.idle_delay_ms = 0;
  state.rx_filtered.throttle = 0.5f;
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void launch_to_active() {
  prepare_launch();
  state.accel_raw.pitch = 2.0f;
  tick(profile.wing.autolaunch.detect_time_ms + 1);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DETECTED, state.wing_launch_state);
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  state.accel_raw.pitch = 0;
  tick(profile.wing.autolaunch.motor_delay_ms - 1);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_SPINUP, state.wing_launch_state);
  tick(profile.wing.autolaunch.spinup_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ACTIVE, state.wing_launch_state);
  TEST_ASSERT_TRUE(state.setpoint.pitch > 0);
  TEST_ASSERT_TRUE(state.pidoutput.pitch > 0);
}

static void test_wing_launch_rejects_short_acceleration_pulse() {
  prepare_launch();
  state.accel_raw.pitch = 2.0f;
  tick(profile.wing.autolaunch.detect_time_ms - 1);
  state.accel_raw.pitch = 0;
  tick(1000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_wing_launch_delay_ramp_and_timed_handoff() {
  launch_to_active();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.wing.autolaunch.throttle, state.throttle);
  tick(profile.wing.autolaunch.timeout_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_FINISH, state.wing_launch_state);
  tick(profile.wing.autolaunch.finish_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DONE, state.wing_launch_state);
  tick();
  TEST_ASSERT_FALSE(state.wing_launch_available);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, (0.5f - 0.05f) * 1.0526316f, state.throttle);
  tick(1000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_IDLE, state.wing_launch_state);
  TEST_ASSERT_FALSE(state.wing_launch_available);
}

static void test_wing_launch_failsafe_stops_motor_and_disarms() {
  launch_to_active();
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  tick(3000);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  TEST_ASSERT_EQUAL(WING_LAUNCH_IDLE, state.wing_launch_state);
}

static void test_wing_launch_zero_level_limit_and_zero_pitch_stays_finite() {
  launch_to_active();
  profile.rate.level_max_angle = 0;
  profile.wing.autolaunch.pitch_angle = 0;
  tick(10);
  TEST_ASSERT_TRUE(finite_float(state.stick_vector.yaw));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, state.setpoint.pitch);
}

static void test_wing_imu_tracks_scripted_bank() {
  state.looptime_autodetect = 1000;
  state.accel_raw = {{0, 0, 1}};
  imu_init();
  for (unsigned i = 0; i < 2000; i++) {
    const float bank = MIN(i, 500U) * (30.0f * DEGTORAD / 500.0f);
    state.accel_raw = {{sinf(bank), 0, cosf(bank)}};
    state.gyro_delta_angle = {{i < 500 ? 30.0f * DEGTORAD / 500.0f : 0, 0, 0}};
    imu_calc();
  }
  // Published attitude is in degrees; PID input uses the normalized gravity vector.
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 30.0f, state.attitude.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, state.GEstG.roll);
}

void run_wing_safety_tests() {
  RUN_TEST(test_wing_angle_corrects_roll_and_pitch_in_both_directions);
  RUN_TEST(test_wing_angle_stick_commands_and_manual_handoff);
  RUN_TEST(test_wing_launch_rejects_short_acceleration_pulse);
  RUN_TEST(test_wing_launch_delay_ramp_and_timed_handoff);
  RUN_TEST(test_wing_launch_failsafe_stops_motor_and_disarms);
  RUN_TEST(test_wing_launch_zero_level_limit_and_zero_pitch_stays_finite);
  RUN_TEST(test_wing_imu_tracks_scripted_bank);
}
