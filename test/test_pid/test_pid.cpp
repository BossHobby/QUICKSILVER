#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

// Include mock helpers
#include "mock_helpers.h"

// Include the PID module
#include "control/control.h"
#include "control/angle_pid.h"
#include "control/pid.h"
#include "core/profile.h"
#include "driver/time.h"
#include "util/util.h"

// Test fixtures
static void pid_setUp(void) {
  // Reset hardware mocks before each test
  mock_hardware_reset_all();

  // Initialize default profile values
  memset(&profile, 0, sizeof(profile));
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));

  // Set reasonable defaults
  profile.pid.pid_rates[0].kp.roll = 1.0f;
  profile.pid.pid_rates[0].kp.pitch = 1.0f;
  profile.pid.pid_rates[0].kp.yaw = 1.0f;

  profile.pid.pid_rates[0].ki.roll = 0.5f;
  profile.pid.pid_rates[0].ki.pitch = 0.5f;
  profile.pid.pid_rates[0].ki.yaw = 0.5f;

  profile.pid.pid_rates[0].kd.roll = 0.1f;
  profile.pid.pid_rates[0].kd.pitch = 0.1f;
  profile.pid.pid_rates[0].kd.yaw = 0.1f;

  // Initialize voltage profile
  profile.voltage.pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_NONE;

  // Initialize control state
  state.looptime = 0.000125f; // 8kHz
  state.looptime_inverse = 1.0f / state.looptime; // 8000Hz
  state.vbat_sag_filtered = 4.0f;
  state.lipo_cell_count = 1;
  state.airspeed = 0.0f; // no airspeed: wing gains apply as tuned

  // Initialize PID
  pid_init();
}


#ifdef VEHICLE_WING
void test_wing_rate_feedforward_tracks_target_while_disarmed() {
  pid_setUp();
  profile.pid.pid_rates[0] = {};
  profile.pid.pid_rates[0].kff = (vec3_t){{44.0f, 30.0f, 10.0f}};
  pid_rates_update();
  state.setpoint = (vec3_t){{1.0f, -0.5f, 0.2f}};
  state.gyro = state.setpoint;
  state.error = {};
  for (int i = 0; i < 100; i++) {
    pid_calc();
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.44f, state.pidoutput.roll);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.15f, state.pidoutput.pitch);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, state.pidoutput.yaw);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_i_term.roll);

  state.setpoint = {};
  // Settle the shared I-term relax filter before the next test.
  for (int i = 0; i < 1000; i++) {
    pid_calc();
  }
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.pitch);
}

void test_wing_rate_feedforward_profile_switch_and_limits() {
  pid_setUp();
  profile.pid.pid_rates[0] = {};
  profile.pid.pid_rates[0].kff = (vec3_t){{100, 100, 100}};
  pid_rates_update();
  state.setpoint = (vec3_t){{10, -10, 10}};
  pid_calc();
  // Wing surfaces use full travel on every axis.
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.pidoutput.roll);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, state.pidoutput.pitch);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.pidoutput.yaw);

  profile.pid.pid_rates[1] = {};
  profile.pid.pid_profile = PID_PROFILE_2;
  pid_rates_update();
  pid_calc();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.yaw);
  state.setpoint = {};
  for (int i = 0; i < 1000; i++) {
    pid_calc();
  }
}
#endif

#ifdef VEHICLE_MULTI
void test_horizon_error_matches_setpoint(void) {
  for (int race = 0; race < 2; race++) {
    pid_setUp();
    profile.pid.small_angle.kp = profile.pid.big_angle.kp = 10.0f;
    profile.pid.small_angle.kd = profile.pid.big_angle.kd = 0.1f;
    state.aux_active = (1U << AUX_LEVELMODE) | (1U << AUX_HORIZON);
    if (race) {
      state.aux_active |= 1U << AUX_RACEMODE;
    }
    state.GEstG = (vec3_t){{0.1f, 0.2f, 0.9746794f}};
    state.gyro = (vec3_t){{0.3f, -0.2f, 0.0f}};
    state.angle_error = (vec3_t){0};
    angle_pid(0);
    angle_pid(1);
    control();
    TEST_ASSERT_GREATER_THAN_FLOAT(0.01f, fabsf(state.angle_error.roll));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, state.setpoint.roll - state.gyro.roll, state.error.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, state.setpoint.pitch - state.gyro.pitch, state.error.pitch);
  }
}
#endif

void test_pid_rates_update_preserves_integral(void) {
  pid_setUp();
  flags.arm_state = flags.in_air = 1;
  profile.pid.pid_rates[0].kd = (vec3_t){0};
  pid_rates_update();
  state.error.roll = 0.1f;
  for (int i = 0; i < 20; i++) {
    pid_calc();
  }
  const float integral = state.pid_i_term.roll;
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, integral);

  profile.pid.pid_rates[1].kp = (vec3_t){{62.8f, 62.8f, 31.4f}};
  profile.pid.pid_rates[1].ki = (vec3_t){0};
  profile.pid.pid_rates[1].kd = (vec3_t){0};
  profile.pid.pid_profile = PID_PROFILE_2;
  pid_rates_update();
  pid_calc();
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.01f, state.pid_p_term.roll);
  TEST_ASSERT_EQUAL_FLOAT(integral, state.pid_i_term.roll);

  profile.pid.pid_rates[1].kp.roll *= 2.0f;
  pid_rates_update();
  pid_calc();
  TEST_ASSERT_FLOAT_WITHIN(1e-7f, 0.02f, state.pid_p_term.roll);
  TEST_ASSERT_EQUAL_FLOAT(integral, state.pid_i_term.roll);
}

void test_pid_filter_update_retains_history_on_period_change(void) {
  pid_setUp();
  state.looptime_autodetect = 125.0f;
  profile.filter.dterm[0].type = FILTER_LP_PT1;
  profile.filter.dterm[0].cutoff_freq = 100.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;
  pid_init();

  state.gyro.roll = 0.001f;
  pid_calc();
  const float omega = 2.0f * M_PI_F * 100.0f;
  const float alpha = omega * state.looptime / (1.0f + omega * state.looptime);
  const float expected = -0.001f / 37500.0f * state.looptime_inverse * alpha;
  TEST_ASSERT_FLOAT_WITHIN(1e-8f, expected, state.pid_d_term.roll);

  state.looptime_autodetect = 250.0f;
  state.looptime = 0.00025f;
  state.looptime_inverse = 4000.0f;
  control_filter_update(false);
  pid_calc(); // No new gyro delta; the existing filter tail must survive.
  const float next_alpha = omega * state.looptime / (1.0f + omega * state.looptime);
  TEST_ASSERT_FLOAT_WITHIN(1e-8f, expected * (1.0f - next_alpha), state.pid_d_term.roll);

  profile.filter.dterm[0].type = FILTER_LP_PT2;
  control_filter_update(true);
  pid_calc();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_d_term.roll);
  state.gyro.roll += 0.001f;
  pid_calc();
  const float pt2_step = omega * 1.55377397403f * state.looptime;
  const float pt2_alpha = pt2_step / (1.0f + pt2_step);
  TEST_ASSERT_FLOAT_WITHIN(1e-8f, -0.001f / 37500.0f * state.looptime_inverse * pt2_alpha * pt2_alpha, state.pid_d_term.roll);
}

// Test basic proportional control
void test_pid_proportional_control(void) {
  pid_setUp();
  // Setup
  state.error.roll = 1.0f; // 1 radian error
  state.error.pitch = 0.5f;
  state.error.yaw = 0.0f;

  // Disable I and D terms for this test
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].ki.pitch = 0.0f;
  profile.pid.pid_rates[0].ki.yaw = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 0.0f;
  profile.pid.pid_rates[0].kd.pitch = 0.0f;
  profile.pid.pid_rates[0].kd.yaw = 0.0f;

  // Reset integral state
  pid_init();

  // Execute
  pid_calc();

  // Verify proportional outputs
  // Note: outputs are scaled by pid_scales
  const float roll_scale = 1.0f / 628.0f;
  const float pitch_scale = 1.0f / 628.0f;
  TEST_ASSERT_EQUAL_FLOAT(1.0f * roll_scale, state.pidoutput.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.5f * pitch_scale, state.pidoutput.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.yaw);
}

// Test integral accumulation
void test_pid_integral_accumulation(void) {
  pid_setUp();
  // Setup - use small error for predictable integral growth
  state.error.roll = 0.1f;
  
  // Enable only integral term
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 1.0f;

  // Reset and initialize
  pid_init();

  // Simulate multiple loops to build up integral
  float expected_integral = 0.0f;
  for (int i = 0; i < 10; i++) {
    pid_calc();
    expected_integral += state.error.roll * state.looptime * profile.pid.pid_rates[0].ki.roll;
  }

  // Verify integral accumulated correctly
  TEST_ASSERT_FLOAT_WITHIN(0.001f, expected_integral, state.pidoutput.roll);
}

// Test derivative calculation - gyro response
void test_pid_derivative_calculation(void) {
  pid_setUp();
  // Setup - only D term enabled
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  // A rising gyro rate is opposed.
  state.gyro.roll = 1.0f;
  pid_calc();
  TEST_ASSERT_TRUE(state.pidoutput.roll < -0.1f);
}

// The D term acts on the gyro alone: a setpoint change does not move it.
void test_pid_dterm_ignores_setpoint(void) {
  pid_setUp();
  profile.pid.pid_rates[0].kp.roll = 0.0f;
  profile.pid.pid_rates[0].ki.roll = 0.0f;
  profile.pid.pid_rates[0].kd.roll = 1.0f;

  state.gyro.roll = 0.0f;
  state.setpoint.roll = 0.0f;
  pid_init();
  pid_calc();

  state.setpoint.roll = 1.0f;
  pid_calc();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_d_term.roll);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.roll);
}

#ifdef VEHICLE_MULTI
// Output from kff alone for a setpoint ramping at the given rate.
static float feedforward_ramp_output(float rate) {
  for (int i = 0; i < 2000; i++) {
    state.setpoint.roll += rate * state.looptime;
    pid_calc();
  }
  return state.pidoutput.roll;
}

static void feedforward_setUp(void) {
  pid_setUp();
  profile.pid.pid_rates[0] = {};
  profile.pid.pid_rates[0].kff.roll = 75.0f;
  state.looptime_autodetect = 125.0f;
  state.rx_filter_hz = 50.0f;
  flags.controls_override = 0;
  state.gyro = {};
  state.setpoint = {};
  state.error = {};
  pid_init();
  pid_calc();
}

void test_pid_feedforward_follows_setpoint_rate(void) {
  feedforward_setUp();
  // kff shares kd's scale: 75 / 37500 output per rad/s^2.
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02f, feedforward_ramp_output(10.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_d_term.roll);

  // A steady setpoint needs none.
  for (int i = 0; i < 2000; i++) {
    pid_calc();
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, state.pidoutput.roll);
}

void test_pid_feedforward_ignores_overrides(void) {
  feedforward_setUp();
  flags.controls_override = 1;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, feedforward_ramp_output(10.0f));

  // Handing back steps the setpoint without any stick movement.
  flags.controls_override = 0;
  state.setpoint.roll = 0.0f;
  pid_calc();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pidoutput.roll);

  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.02f, feedforward_ramp_output(10.0f));
}
#endif

// Test voltage compensation
void test_pid_voltage_compensation(void) {
  pid_setUp();
  // Enable voltage compensation
  profile.voltage.pid_voltage_compensation = PID_VOLTAGE_COMPENSATION_ACTIVE;
  
  state.error.roll = 1.0f;
  state.lipo_cell_count = 4;

  // A full pack needs no boost.
  state.vbat_sag_filtered = 16.0f;
  pid_calc();
  const float full_p_term = state.pid_p_term.roll;
  TEST_ASSERT_NOT_EQUAL(0.0f, full_p_term);

  // Sag under load raises P immediately; the slow display voltage is ignored.
  state.vbat_sag_filtered = 12.0f;
  state.vbat_cell_avg = 4.0f;
  pid_calc();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.2078f, state.pid_p_term.roll / full_p_term);
}

// Test complete PID loop
void test_pid_complete_loop(void) {
  pid_setUp();
  // Setup realistic values
  state.error.roll = 0.5f;
  state.error.pitch = -0.3f;
  state.error.yaw = 0.1f;

  pid_init();
  
  // Run for several iterations
  for (int i = 0; i < 5; i++) {
    pid_calc();
    
    // Verify outputs are within reasonable range
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.roll);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.pitch);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 0.0f, state.pidoutput.yaw);
    
    // Simulate changing errors
    state.error.roll *= 0.9f;
    state.error.pitch *= 0.9f;
    state.error.yaw *= 0.9f;
  }
}
void test_angle_pid_uses_legacy_dterm_timefactor(void) {
  pid_setUp();

  profile.pid.small_angle.kp = 10.0f;
  profile.pid.small_angle.kd = 1.0f;
  profile.pid.big_angle.kp = 0.0f;
  profile.pid.big_angle.kd = 0.0f;

  state.angle_error.roll = 0.0f;
  angle_pid(0);

  state.angle_error.roll = 0.1f;
  const float output = angle_pid(0);

  const float p_term = 0.1f * 10.0f * (1.0f - 0.1f);
  const float d_term = 0.1f * 1.0f * (1.0f - 0.1f) * (0.0032f * state.looptime_inverse);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, p_term + d_term, output);
}

void test_angle_pid_rth_damps_rotation_without_target_kick(void) {
  pid_setUp();
  state.nav_state = NAV_STATE_CLIMB;
  profile.pid.small_angle.kp = profile.pid.big_angle.kp = 1;
  profile.pid.small_angle.kd = profile.pid.big_angle.kd = 3;
  for (int axis = 0; axis < 2; axis++) {
    state.angle_error.axis[axis] = 0;
    angle_pid(axis);
    state.angle_error.axis[axis] = 0.2f;
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.2f, angle_pid(axis));
    state.angle_error.axis[axis] = -0.2f;
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -0.2f, angle_pid(axis));
    state.angle_error.axis[axis] = 0;
    state.gyro.axis[axis] = 1;
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -0.0096f, angle_pid(axis));
    state.gyro.axis[axis] = -1;
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.0096f, angle_pid(axis));
    state.gyro.axis[axis] = 0;
    state.nav_state = NAV_STATE_INACTIVE;
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0, angle_pid(axis));
    state.nav_state = NAV_STATE_CLIMB;
  }
}
