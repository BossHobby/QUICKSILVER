#include <bit>
#include <math.h>
#include <unity.h>

#include "control/imu.h"

#include "control/control.h"
#include "control/navigation.h"
#include "control/pid.h"
#include "core/profile.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "io/gps.h"
#include "mock_outputs.h"
#include "util/util.h"

extern void imu_test_attitude_init();
extern void imu_test_attitude_update();

extern const profile_t default_profile;

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
  profile_set_defaults(&profile); // Use the same startup defaults as hardware.
  profile_output_update();
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

static void assert_launch_stabilized() {
  TEST_ASSERT_TRUE(state.setpoint.roll < 0);
  TEST_ASSERT_TRUE(state.pidoutput.roll < 0);
  // Positive pitch is nose-down: a level wing pitches up towards the climb angle.
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0);
  TEST_ASSERT_TRUE(state.pidoutput.pitch < 0);
}

// Specific force along accel_raw.pitch, which reads positive nose-down at rest.
#define FORWARD_THROW (-2.0f)
#define BACKWARD_JERK (2.0f)

static void test_wing_launch_stabilizes_while_waiting_and_releases_on_switch_off() {
  prepare(1U << AUX_AUTOLAUNCH);
  state.GEstG = {{0.5f, 0, 0.8660254f}};
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_IDLE, state.wing_launch_state);
  assert_launch_stabilized();
  state.rx_filtered.throttle = 0.5f;
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_IDLE_DELAY, state.wing_launch_state);
  assert_launch_stabilized();
  tick(1500);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  assert_launch_stabilized();
  state.aux_active |= 1U << AUX_ACROMODE;
  tick();
  assert_launch_stabilized();
  state.aux_active |= 1U << AUX_LEVELMODE;
  tick();
  assert_launch_stabilized();
  state.aux_active &= ~((1U << AUX_AUTOLAUNCH) | (1U << AUX_ACROMODE) | (1U << AUX_LEVELMODE));
  tick();
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[1]);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[2]);
}

static void launch_to_active() {
  prepare_launch();
  state.GEstG = {{0.5f, 0, 0.8660254f}};
  state.accel_raw.pitch = FORWARD_THROW;
  tick(profile.wing.autolaunch.detect_time_ms + 1);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DETECTED, state.wing_launch_state);
  assert_launch_stabilized();
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  assert_launch_stabilized();
  state.accel_raw.pitch = 0;
  tick(); // Release starts the motor delay.
  tick(profile.wing.autolaunch.motor_delay_ms - 1);
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_SPINUP, state.wing_launch_state);
  assert_launch_stabilized();
  tick(profile.wing.autolaunch.spinup_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ACTIVE, state.wing_launch_state);
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0);
  TEST_ASSERT_TRUE(state.pidoutput.pitch < 0);
}

static void test_wing_launch_motor_delay_starts_at_release() {
  prepare_launch();
  state.accel_raw.pitch = FORWARD_THROW;
  tick(profile.wing.autolaunch.detect_time_ms + 2);
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  // Still in the hand: a long throw must not spend the delay.
  tick(300);
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  state.accel_raw.pitch = 0;
  tick(profile.wing.autolaunch.motor_delay_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_SPINUP, state.wing_launch_state);
}

static void test_wing_launch_sustained_acceleration_counts_as_released() {
  prepare_launch();
  state.accel_raw.pitch = FORWARD_THROW;
  tick(profile.wing.autolaunch.detect_time_ms + 2);
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  tick(500 + profile.wing.autolaunch.motor_delay_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_SPINUP, state.wing_launch_state);
}

static void test_wing_launch_rejects_short_acceleration_pulse() {
  prepare_launch();
  state.accel_raw.pitch = FORWARD_THROW;
  tick(profile.wing.autolaunch.detect_time_ms - 1);
  state.accel_raw.pitch = 0;
  tick(1000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_wing_launch_ignores_backward_jerk() {
  prepare_launch();
  state.accel_raw.pitch = BACKWARD_JERK; // Wind-up or catching the airframe.
  tick(1000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_wing_launch_delay_ramp_and_timed_handoff() {
  launch_to_active();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.wing.autolaunch.throttle, state.throttle);
  tick(profile.wing.autolaunch.timeout_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_FINISH, state.wing_launch_state);
  assert_launch_stabilized();
  tick(profile.wing.autolaunch.finish_ms);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DONE, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[1]);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[2]);
  tick();
  TEST_ASSERT_FALSE(state.wing_launch_available);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, (0.5f - 0.05f) * 1.0526316f, state.throttle);
  tick(1000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DONE, state.wing_launch_state);
  TEST_ASSERT_FALSE(state.wing_launch_available);
}

static void test_wing_launch_failsafe_stops_motor_and_disarms() {
  launch_to_active();
  profile.navigation.rth_on_failsafe = false;
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  tick(3000);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  TEST_ASSERT_EQUAL(WING_LAUNCH_IDLE, state.wing_launch_state);
}

static void test_wing_launch_abort_returns_surfaces_to_manual() {
  launch_to_active();
  state.rx_filtered.roll = 0.2f;
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_ABORTED, state.wing_launch_state);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, pwm_values[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.2f, pwm_values[2]);
}

static void test_wing_launch_stage1_recovery_requires_disarm_before_relaunch() {
  launch_to_active();
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  tick(FAILSAFE_HOLD_TIME_US / 1000 - 1);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ACTIVE, state.wing_launch_state);
  TEST_ASSERT_TRUE(state.wing_launch_available);
  tick(1);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ABORTED, state.wing_launch_state);
  TEST_ASSERT_FALSE(state.wing_launch_available);

  flags.failsafe_signal_lost = 0;
  state.rx_filtered.throttle = 0.5f;
  state.accel_raw.pitch = FORWARD_THROW; // Even a fresh launch trigger must be ignored.
  tick(2000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_FALSE(state.wing_launch_available);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ABORTED, state.wing_launch_state);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, (0.5f - 0.05f) * 1.0526316f, state.throttle);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[1]);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[2]);

  state.aux_active &= ~(1U << AUX_ARMING);
  state.rx_filtered.throttle = 0;
  state.accel_raw.pitch = 0;
  tick(2);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(state.wing_launch_available);
  state.aux_active |= 1U << AUX_ARMING;
  tick();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 0.5f;
  tick();
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
}

static void test_wing_launch_switch_reenabled_in_flight_does_not_restart() {
  prepare_launch();
  state.aux_active &= ~(1U << AUX_AUTOLAUNCH);
  tick();
  TEST_ASSERT_FALSE(state.wing_launch_available);
  state.aux_active |= 1U << AUX_AUTOLAUNCH;
  state.GEstG = {{0.5f, 0, 0.8660254f}};
  tick(2000);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ABORTED, state.wing_launch_state);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, (0.5f - 0.05f) * 1.0526316f, state.throttle);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[1]);
  TEST_ASSERT_EQUAL_FLOAT(0, pwm_values[2]);
}

static void test_wing_launch_gps_requires_launch_attitude() {
  prepare_launch();
  state.gps_lock = 1;
  state.gps_speed = profile.wing.autolaunch.velocity_threshold + 1.0f;
  state.attitude.pitch = 60.0f * DEGTORAD; // Nose down.
  tick(100);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  state.attitude.pitch = -60.0f * DEGTORAD; // Steeply nose up.
  tick(100);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  state.attitude.pitch = -15.0f * DEGTORAD;
  tick(profile.wing.autolaunch.detect_time_ms + 1);
  TEST_ASSERT_EQUAL(WING_LAUNCH_DETECTED, state.wing_launch_state);
}

static void test_wing_launch_pitch_exceeds_level_limit() {
  prepare_launch();
  profile.rate.level_max_angle = 10.0f;
  profile.wing.autolaunch.pitch_angle = 20.0f;
  const float pitch = -15.0f * DEGTORAD; // Nose-up between the two limits.
  state.GEstG = {{0, sinf(pitch), cosf(pitch)}};
  tick();
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0); // Keep climbing towards 20 degrees.
}

static void test_wing_failsafe_stage1_levels_manual_mode() {
  prepare(0);
  profile.navigation.rth_on_failsafe = false;
  state.rx_filtered.throttle = 0.5f;
  tick();
  state.GEstG = {{0.5f, 0, 0.8660254f}};
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  tick(FAILSAFE_HOLD_TIME_US / 1000 + 1);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL_STRING("FS LEVEL", control_flight_mode_name());
  TEST_ASSERT_TRUE(state.setpoint.roll < 0);
  TEST_ASSERT_TRUE(state.pidoutput.roll < 0);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
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
  imu_test_attitude_init();
  for (unsigned i = 0; i < 2000; i++) {
    const float bank = MIN(i, 500U) * (30.0f * DEGTORAD / 500.0f);
    state.accel_raw = {{sinf(bank), 0, cosf(bank)}};
    state.gyro_delta_angle = {{i < 500 ? 30.0f * DEGTORAD / 500.0f : 0, 0, 0}};
    state.gyro.roll = state.gyro_delta_angle.roll * state.looptime_inverse;
    imu_calc();
  }
  // The attitude task publishes radians; PID input uses the gravity vector.
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 30.0f * DEGTORAD, state.attitude.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, state.GEstG.roll);
}

static void test_wing_default_rate_response_across_profiles_and_loop_times() {
  prepare(1U << AUX_ACROMODE);
  flags.in_air = 1;
  const uint32_t periods_us[] = {250, 1000, 2000};
  for (uint8_t bank = 0; bank < PID_PROFILE_MAX; bank++) {
    // Configurator's default-profile response must agree with boot defaults.
    TEST_ASSERT_EQUAL_MEMORY(&profile.pid.pid_rates[bank], &default_profile.pid.pid_rates[bank], sizeof(pid_rate_t));
    profile.pid.pid_profile = (pid_profile_t)bank;
    for (uint32_t period_us : periods_us) {
      state.looptime = period_us * 0.000001f;
      state.looptime_inverse = 1.0f / state.looptime;
      state.looptime_autodetect = period_us;
      pid_init();
      state.error = {{1, 1, 1}}; // Sustained 57.3 deg/s rate error.
      for (uint32_t elapsed_us = 0; elapsed_us < 1000000; elapsed_us += period_us)
        pid_calc();
      // Modest initial response plus slow roll/pitch integration, not saturation.
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.302f, state.pidoutput.roll);
      TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.251f, state.pidoutput.pitch);
      TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.032f, state.pidoutput.yaw);
      TEST_ASSERT_EQUAL_FLOAT(0, state.pid_d_term.roll);
      TEST_ASSERT_EQUAL_FLOAT(0, state.pid_d_term.pitch);
      TEST_ASSERT_EQUAL_FLOAT(0, state.pid_i_term.yaw);
    }
  }
}

static void test_wing_gps_home_latches_on_arm() {
  prepare(0);
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  state.gps_lock = true;
  state.gps_sats = GPS_MIN_SATS_FOR_LOCK;
  state.gps_horizontal_accuracy = 1.0f;
  state.gps_coord = {.lon = 1142000000, .lat = 223000000};
  time_test_advance_us(10000);
  state.gps_last_update_ms = time_millis();
  nav_update(); // First armed pass latches home.
  TEST_ASSERT_EQUAL_INT32(223000000, state.gps_home.lat);

  state.gps_coord.lat += 899; // About 10 m north.
  time_test_advance_us(10000);
  state.gps_last_update_ms = time_millis();
  nav_update();
  TEST_ASSERT_EQUAL_INT32(223000000, state.gps_home.lat);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 10.0f, state.home_distance);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 180.0f, state.home_bearing);
}

static void test_wing_default_level_response_has_no_derivative_kick_or_voltage_boost() {
  prepare(1U << AUX_LEVELMODE);
  state.GEstG = {{0.5f, 0, 0.8660254f}}; // 30-degree bank, zero angular rate.
  tick();
  const float first_target = state.setpoint.roll;
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -1.5f, first_target);
  tick();
  TEST_ASSERT_EQUAL_FLOAT(first_target, state.setpoint.roll);
  const float first_p = state.pid_p_term.roll;
  state.vbat_cell_avg = 2.5f;
  tick();
  TEST_ASSERT_EQUAL_FLOAT(first_p, state.pid_p_term.roll);
}

void run_wing_safety_tests() {
  RUN_TEST(test_wing_angle_corrects_roll_and_pitch_in_both_directions);
  RUN_TEST(test_wing_angle_stick_commands_and_manual_handoff);
  RUN_TEST(test_wing_launch_stabilizes_while_waiting_and_releases_on_switch_off);
  RUN_TEST(test_wing_launch_motor_delay_starts_at_release);
  RUN_TEST(test_wing_launch_sustained_acceleration_counts_as_released);
  RUN_TEST(test_wing_launch_rejects_short_acceleration_pulse);
  RUN_TEST(test_wing_launch_ignores_backward_jerk);
  RUN_TEST(test_wing_launch_delay_ramp_and_timed_handoff);
  RUN_TEST(test_wing_launch_failsafe_stops_motor_and_disarms);
  RUN_TEST(test_wing_launch_abort_returns_surfaces_to_manual);
  RUN_TEST(test_wing_launch_stage1_recovery_requires_disarm_before_relaunch);
  RUN_TEST(test_wing_launch_switch_reenabled_in_flight_does_not_restart);
  RUN_TEST(test_wing_launch_gps_requires_launch_attitude);
  RUN_TEST(test_wing_launch_pitch_exceeds_level_limit);
  RUN_TEST(test_wing_failsafe_stage1_levels_manual_mode);
  RUN_TEST(test_wing_launch_zero_level_limit_and_zero_pitch_stays_finite);
  RUN_TEST(test_wing_imu_tracks_scripted_bank);
  RUN_TEST(test_wing_default_rate_response_across_profiles_and_loop_times);
  RUN_TEST(test_wing_gps_home_latches_on_arm);
  RUN_TEST(test_wing_default_level_response_has_no_derivative_kick_or_voltage_boost);
}
