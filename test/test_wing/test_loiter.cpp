#include <math.h>
#include <unity.h>

#include "control/control.h"
#include "control/navigation.h"
#include "control/wing/navigation.h"
#include "core/profile.h"
#include "driver/time.h"
#include "io/gps.h"
#include "util/util.h"

// Script GPS fixes and run Flight's control/navigation order. This checks
// guidance direction and transitions, not airframe dynamics.
static constexpr gps_coord_t ORIGIN = {.lon = 1142000000, .lat = 223000000};

static void set_gps(float north, float east, float vel_north, float vel_east) {
  state.gps_lock = true;
  state.gps_sats = GPS_MIN_SATS_FOR_LOCK;
  state.gps_horizontal_accuracy = 1.0f;
  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  state.gps_coord.lat = ORIGIN.lat + (int32_t)lrintf(north / METERS_PER_DEGREE_LAT * 1e7f);
  state.gps_coord.lon = ORIGIN.lon + (int32_t)lrintf(east / lon_scale * 1e7f);
  state.gps_vel_north = vel_north;
  state.gps_vel_east = vel_east;
  state.gps_speed = hypotf(vel_north, vel_east);
  state.gps_last_update_ms = time_millis();
}

static void step(uint32_t milliseconds = 10) {
  for (uint32_t i = 0; i < milliseconds; i += 10) {
    time_test_advance_us(10000);
    if (state.gps_lock)
      state.gps_last_update_ms = time_millis();
    control();
    nav_update();
  }
}

static void fly(uint32_t aux) {
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  set_gps(0.0f, 0.0f, 0.0f, 15.0f); // East at 15 m/s.
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | aux;
  step();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 0.5f;
  step();
  TEST_ASSERT_TRUE(flags.in_air);
}

static float roll_command_deg() {
  return wing_nav_command.roll * RADTODEG;
}

static void test_loiter_requires_switch_and_flight() {
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | (1U << AUX_LOITER);
  step();
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_FALSE(flags.in_air);
  TEST_ASSERT_EQUAL(WING_LOITER_INACTIVE, state.wing_loiter_state);

  state.rx_filtered.throttle = 0.5f;
  step(20);
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);
  TEST_ASSERT_EQUAL_STRING("LOITER", control_flight_mode_name());

  state.aux_active &= ~(1U << AUX_LOITER);
  step();
  TEST_ASSERT_EQUAL(WING_LOITER_INACTIVE, state.wing_loiter_state);
  TEST_ASSERT_EQUAL_STRING("MANUAL", control_flight_mode_name());
}

static void test_loiter_enters_tangentially_on_turn_side() {
  fly(1U << AUX_LOITER);
  step(1000);
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);
  // On the circle at the entry point: bank for v^2/r and hold it.
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.wing.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.wing.navigation.cruise_throttle, state.throttle);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.wing.navigation.cruise_throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);
}

static void test_loiter_left_direction_banks_left() {
  profile.wing.navigation.loiter_direction = WING_LOITER_LEFT;
  fly(1U << AUX_LOITER);
  step(1000);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.wing.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll < 0.0f);
}

static void test_loiter_turns_back_when_flying_away() {
  fly(1U << AUX_LOITER);
  // The circle center is 50 m south of the entry point. Fly north away from it.
  for (int north = 40; north <= 200; north += 40) {
    set_gps((float)north, 0.0f, 15.0f, 0.0f);
    step(100);
  }
  step(1000);
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, profile.wing.navigation.max_bank_angle, roll_command_deg());

  // Heading back toward the center needs no further turn.
  set_gps(200.0f, 0.0f, -15.0f, 0.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, roll_command_deg());
}

static void test_loiter_banks_without_gps_and_recenters_on_recovery() {
  fly(1U << AUX_LOITER);
  state.gps_lock = false;
  step(1000);
  TEST_ASSERT_EQUAL(WING_LOITER_BANK, state.wing_loiter_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, roll_command_deg());

  // The aircraft drifted; the circle is placed again from the new fix.
  set_gps(30.0f, 30.0f, 15.0f, 0.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.wing.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
}

static void test_loiter_runs_without_configured_gps() {
  fly(1U << AUX_LOITER);
  profile.serial.gps = SERIAL_PORT_INVALID;
  step(1000);
  TEST_ASSERT_EQUAL(WING_LOITER_BANK, state.wing_loiter_state);
}

static void test_loiter_yields_to_failsafe_and_disarm() {
  fly(1U << AUX_LOITER);
  step(100);
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);

  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  step(400);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_LOITER_INACTIVE, state.wing_loiter_state);
  TEST_ASSERT_EQUAL_STRING("FS LEVEL", control_flight_mode_name());

  flags.failsafe_signal_lost = 0;
  step();
  TEST_ASSERT_EQUAL(WING_LOITER_CIRCLE, state.wing_loiter_state);

  state.aux_active &= ~(1U << AUX_ARMING);
  step();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL(WING_LOITER_INACTIVE, state.wing_loiter_state);
}

static void test_loiter_waits_for_autolaunch() {
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  set_gps(0.0f, 0.0f, 0.0f, 0.0f);
  state.aux_active = (1U << AUX_PREARM) | (1U << AUX_AUTOLAUNCH) | (1U << AUX_LOITER);
  step();
  state.aux_active |= 1U << AUX_ARMING;
  step();
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_TRUE(state.wing_launch_available);
  state.rx_filtered.throttle = 0.5f;
  step(100);
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_TRUE(state.wing_launch_state < WING_LAUNCH_DONE);
  TEST_ASSERT_EQUAL(WING_LOITER_INACTIVE, state.wing_loiter_state);
}

void run_wing_loiter_tests() {
  RUN_TEST(test_loiter_requires_switch_and_flight);
  RUN_TEST(test_loiter_enters_tangentially_on_turn_side);
  RUN_TEST(test_loiter_left_direction_banks_left);
  RUN_TEST(test_loiter_turns_back_when_flying_away);
  RUN_TEST(test_loiter_banks_without_gps_and_recenters_on_recovery);
  RUN_TEST(test_loiter_runs_without_configured_gps);
  RUN_TEST(test_loiter_yields_to_failsafe_and_disarm);
  RUN_TEST(test_loiter_waits_for_autolaunch);
}
