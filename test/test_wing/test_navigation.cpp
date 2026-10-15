#include <math.h>
#include <unity.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/mission.h"
#include "control/navigation.h"
#include "control/wing/navigation.h"
#include "core/profile.h"
#include "driver/baro/baro.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "io/gps.h"
#include "util/util.h"

extern void imu_test_attitude_init();

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
    if (state.baro_valid)
      state.baro_last_update_ms = time_millis();
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
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);

  state.rx_filtered.throttle = 0.5f;
  step(20);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("LOITER", control_flight_mode_name());

  state.aux_active &= ~(1U << AUX_LOITER);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("MANUAL", control_flight_mode_name());
}

static void test_loiter_enters_tangentially_on_turn_side() {
  fly(1U << AUX_LOITER);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  // On the circle at the entry point: bank for v^2/r and hold it.
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.throttle);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);
}

static void test_loiter_left_direction_banks_left() {
  profile.navigation.loiter_direction = NAV_LOITER_LEFT;
  fly(1U << AUX_LOITER);
  step(1000);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll < 0.0f);
}

static void test_loiter_turns_back_when_flying_away() {
  fly(1U << AUX_LOITER);
  // The circle center is one loiter radius south of the entry point. Fly north away from it.
  for (int north = 40; north <= 200; north += 40) {
    set_gps((float)north, 0.0f, 15.0f, 0.0f);
    step(100);
  }
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, profile.navigation.max_bank_angle, roll_command_deg());

  // Heading back toward the center needs no further turn.
  set_gps(200.0f, 0.0f, -15.0f, 0.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, roll_command_deg());
}

static void test_loiter_caps_configured_bank() {
  profile.navigation.max_bank_angle = 60.0f;
  fly(1U << AUX_LOITER);
  for (int north = 40; north <= 200; north += 40) {
    set_gps((float)north, 0.0f, 15.0f, 0.0f);
    step(100);
  }
  step(1000);
  // Steeper sustained turns exceed the IMU's gravity-correction limit.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 40.0f, roll_command_deg());
}

static void test_loiter_banks_without_gps_and_recenters_on_recovery() {
  fly(1U << AUX_LOITER);
  state.gps_lock = false;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, roll_command_deg());

  // The aircraft drifted; the circle is placed again from the new fix.
  set_gps(30.0f, 30.0f, 15.0f, 0.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
}

// Turn for 20 s at the given bank and body rates with the accelerometer reading
// accel_raw, airborne at the cruise speed. Returns the angle between the
// gravity estimate level control uses and true gravity at the end, in degrees.
static float turn_gravity_error_deg(float bank, vec3_t gyro, vec3_t accel_raw) {
  state.looptime_autodetect = 1000;
  state.accel_raw = {{sinf(bank), 0, cosf(bank)}};
  imu_init();
  imu_test_attitude_init();
  flags.arm_state = 1;
  flags.in_air = 1;
  state.wing_speed_scale = 1.0f;
  imu_calc();

  state.accel_raw = accel_raw;
  state.gyro = gyro;
  state.gyro_delta_angle = vec3_mul(state.gyro, state.looptime);
  for (uint32_t ms = 1; ms <= 20000; ms++) {
    time_test_advance_us(1000);
    imu_calc();
  }
  const vec3_t gravity = {{sinf(bank), 0, cosf(bank)}};
  return acosf(constrain(vec3_dot(state.GEstG, gravity), -1.0f, 1.0f)) * RADTODEG;
}

// A coordinated right turn at 35 degrees bank and 20 m/s, with no GPS and no
// trusted heading: the accelerometer reads the whole load along body up.
static float coordinated_turn_gravity_error_deg() {
  const float bank = 35.0f * DEGTORAD;
  profile.navigation.cruise_speed = 20.0f;
  const float turn_rate = 9.80665f * tanf(bank) / profile.navigation.cruise_speed;
  const vec3_t gyro = {{0, -turn_rate * sinf(bank), turn_rate * cosf(bank)}};
  return turn_gravity_error_deg(bank, gyro, (vec3_t){{0, 0, 1.0f / cosf(bank)}});
}

static void test_turn_acceleration_keeps_imu_attitude() {
  TEST_ASSERT_TRUE(coordinated_turn_gravity_error_deg() < 3.0f);
  // Held level for a throw and turned on the spot, the wing reads only gravity.
  state.wing_launch_available = true;
  state.wing_launch_state = WING_LAUNCH_WAIT;
  TEST_ASSERT_TRUE(turn_gravity_error_deg(0.0f, (vec3_t){{0, 0, 0.3f}}, (vec3_t){{0, 0, 1.0f}}) < 1.0f);
  // Uncompensated, the coordinated turn levels the estimate.
  TEST_ASSERT_TRUE(coordinated_turn_gravity_error_deg() > 10.0f);
}

static void test_loiter_runs_without_configured_gps() {
  fly(1U << AUX_LOITER);
  profile.serial.gps = SERIAL_PORT_INVALID;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
}

static void test_loiter_yields_to_failsafe_and_disarm() {
  fly(1U << AUX_LOITER);
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);

  // Without failsafe RTH the existing level glide and stage 2 drop apply.
  profile.navigation.rth_on_failsafe = false;
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  step(400);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.wing_nav_failsafe);
  TEST_ASSERT_EQUAL_STRING("FS LEVEL", control_flight_mode_name());

  flags.failsafe_signal_lost = 0;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);

  state.aux_active &= ~(1U << AUX_ARMING);
  step();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
}

static void test_failsafe_drops_without_failsafe_rth() {
  profile.navigation.rth_on_failsafe = false;
  fly(1U << AUX_LOITER);
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  step(1500);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_FALSE(flags.arm_state);
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
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
}

static void fly_with_baro(float altitude) {
  fly(0);
  // Navigation keeps scripted altitude while armed without new baro samples.
  state.baro_valid = true;
  state.altitude = altitude;
  state.baro_vertical_speed = 0.0f;
  state.aux_active |= 1U << AUX_LOITER;
  step(1000);
}

static void test_loiter_holds_entry_altitude() {
  fly_with_baro(50.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, wing_nav_command.throttle);

  // Below the target: nose up, with throttle added for the climb.
  state.altitude = 45.0f;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.pitch >= -20.0f * DEGTORAD - 0.001f);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > profile.navigation.cruise_throttle);
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0.0f);

  // Above the target: nose down, with throttle reduced for the dive.
  state.altitude = 55.0f;
  step(4000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch > 5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.pitch <= 15.0f * DEGTORAD + 0.001f);
  TEST_ASSERT_TRUE(wing_nav_command.throttle < profile.navigation.cruise_throttle);
}

static void test_loiter_integral_trims_steady_sink() {
  fly_with_baro(50.0f);
  // A steady sink at the target altitude, as in a bank without extra lift.
  state.baro_vertical_speed = -1.0f;
  step(5000);
  const float trimmed = wing_nav_command.pitch;
  TEST_ASSERT_TRUE(trimmed < -3.0f * DEGTORAD);
  state.baro_vertical_speed = 0.0f;
  step(1000);
  // The learned nose-up trim remains once the sink has stopped.
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -1.0f * DEGTORAD);
}

static void test_loiter_levels_pitch_when_baro_goes_stale() {
  fly_with_baro(50.0f);
  state.altitude = 40.0f;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < 0.0f);
  state.baro_valid = false;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, wing_nav_command.throttle);

  // A fresh barometer latches the current altitude rather than chasing the old target.
  state.baro_valid = true;
  step(1000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);
}

// Home latches at the origin on arming. Flies north away from it in fixes
// small enough for the GPS jump check.
static void fly_north_of_home(float distance) {
  fly(0);
  for (float north = 40.0f; north < distance; north += 40.0f) {
    set_gps(north, 0.0f, 15.0f, 0.0f);
    step(100);
  }
  set_gps(distance, 0.0f, 15.0f, 0.0f);
  step(100);
}

static void lose_link() {
  state.last_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
}

static void wait_for_launch() {
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  profile.wing.autolaunch.idle_throttle = 0.0f;
  set_gps(0.0f, 0.0f, 0.0f, 0.0f);
  state.aux_active = (1U << AUX_PREARM) | (1U << AUX_AUTOLAUNCH);
  step();
  state.aux_active |= 1U << AUX_ARMING;
  step();
  state.rx_filtered.throttle = 0.5f;
  step(100);
  TEST_ASSERT_EQUAL(WING_LAUNCH_WAIT, state.wing_launch_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_failsafe_before_throw_disarms_without_rth() {
  wait_for_launch();
  lose_link();
  step(400);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  step(1500);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  // A recovered link and the still-held arm switch must not restart launch.
  flags.failsafe_signal_lost = 0;
  step(600);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_failsafe_during_launch_delay_cannot_start_rth_motor() {
  wait_for_launch();
  profile.wing.autolaunch.motor_delay_ms = 1000;
  state.accel_raw.pitch = -2.0f * ACC_1G;
  step(100);
  state.accel_raw.pitch = 0.0f;
  step();
  TEST_ASSERT_EQUAL(WING_LAUNCH_MOTOR_DELAY, state.wing_launch_state);
  lose_link();
  step(400);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
}

static void test_failsafe_after_launch_spinup_can_return() {
  wait_for_launch();
  state.accel_raw.pitch = -2.0f * ACC_1G;
  step(100);
  state.accel_raw.pitch = 0.0f;
  step(300);
  TEST_ASSERT_EQUAL(WING_LAUNCH_ACTIVE, state.wing_launch_state);
  lose_link();
  step(1500);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HOME, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.throttle > 0.0f);
}

static void test_failsafe_rth_preserves_throttle_during_handoff() {
  fly_north_of_home(200.0f);
  const float throttle = state.throttle;
  TEST_ASSERT_TRUE(throttle > 0.4f);
  lose_link();
  step(FAILSAFE_HOLD_TIME_US / 1000 - 10);
  // Flight continues between navigation updates. Neither stage 1 entry nor
  // the following control passes may seed navigation from zero throttle.
  for (uint32_t i = 0; i < 20; i++) {
    time_test_advance_us(1000);
    control();
    nav_update();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, throttle, state.throttle);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);
  }
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, throttle, wing_nav_command.throttle);
}

static void test_rth_turns_toward_home() {
  fly_north_of_home(200.0f);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, profile.navigation.max_bank_angle, fabsf(roll_command_deg()));

  // Flying west with home to the south: turn left.
  set_gps(200.0f, 0.0f, 0.0f, -15.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -profile.navigation.max_bank_angle, roll_command_deg());

  // Flying straight at home needs no turn.
  set_gps(200.0f, 0.0f, -15.0f, 0.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, roll_command_deg());
}

static void test_rth_circles_home_on_arrival() {
  fly(0);
  const float radius = profile.navigation.loiter_radius;
  // Move out in fixes the GPS position-jump check accepts.
  for (float north = 40.0f; north < radius; north += 40.0f) {
    set_gps(north, 0.0f, 0.0f, 15.0f);
    step(100);
  }
  // Arrival latches inside the circle, independent of rounding at its edge.
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  set_gps(radius, 0.0f, 0.0f, 15.0f); // On the home circle, flying clockwise.
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HOME, state.wing_nav_state);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
}

static void test_rth_climbs_by_return_altitude() {
  fly(0);
  state.baro_valid = true;
  state.altitude = 20.0f;
  state.baro_vertical_speed = 0.0f;
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > profile.navigation.cruise_throttle);
}

static void test_rth_retains_altitude_target_across_baro_outages() {
  fly(0);
  state.baro_valid = true;
  state.altitude = 20.0f;
  state.baro_vertical_speed = 0.0f;
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step();
  state.altitude = 20.0f + profile.navigation.rth_altitude;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  state.baro_valid = false;
  step(1000);
  state.baro_valid = true;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  // Staleness follows the same path, including after manual RTH becomes failsafe.
  lose_link();
  step(400);
  time_test_advance_us((BARO_STALE_MS + 10) * 1000);
  state.gps_last_update_ms = time_millis();
  control();
  nav_update();
  step(2000);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);
}

static void test_rth_latches_altitude_when_baro_first_becomes_valid() {
  fly(0);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(1000);
  state.baro_valid = true;
  state.altitude = 20.0f;
  step();
  state.altitude = 20.0f + profile.navigation.rth_altitude;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  // A new RTH request still gets its own altitude offset.
  state.aux_active &= ~(1U << AUX_RETURN_TO_HOME);
  step();
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
}

static void test_rth_without_home_loiters_in_place() {
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  state.gps_lock = false; // No fix on arming: no home.
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  step();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 0.5f;
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(100);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
}

static void test_rth_steers_imu_heading_for_ten_seconds_after_gps_loss() {
  fly_north_of_home(200.0f);
  set_gps(200.0f, 0.0f, -15.0f, 0.0f);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);

  state.heading_confidence = 1.0f;
  state.heading = 90.0f; // East, with home to the south: turn right.
  state.gps_lock = false;
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HEADING, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, profile.navigation.max_bank_angle, roll_command_deg());

  step(8500);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, roll_command_deg());
}

static void test_rth_banks_at_once_without_heading_confidence() {
  fly_north_of_home(200.0f);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  state.heading_confidence = 0.0f;
  state.gps_lock = false;
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
}

static void test_failsafe_rth_keeps_outputs_and_returns_control() {
  fly_north_of_home(200.0f);
  lose_link();
  step(3000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("FS RTH", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, wing_nav_command.throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);

  flags.failsafe_signal_lost = 0;
  step(20);
  TEST_ASSERT_FALSE(state.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("MANUAL", control_flight_mode_name());
}

static void test_failsafe_rth_continues_on_switch_after_recovery() {
  fly_north_of_home(200.0f);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  lose_link();
  step(2000);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  flags.failsafe_signal_lost = 0;
  step(20);
  TEST_ASSERT_FALSE(state.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
}

static void test_failsafe_without_gps_descends_in_circle() {
  fly(0);
  state.baro_valid = true;
  state.altitude = 50.0f;
  state.baro_vertical_speed = -4.0f; // A glide must not pitch up to chase a powered sink-rate target.
  state.gps_lock = false;
  lose_link();
  step(3000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("FS DESCEND", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, roll_command_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 5.0f * DEGTORAD, wing_nav_command.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wing_nav_command.throttle);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  // Reaching rest must never restart the motor, even without a barometer.
  state.altitude = 0.0f;
  state.baro_vertical_speed = 0.0f;
  step(10000);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  state.baro_valid = false;
  step(10000);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  TEST_ASSERT_TRUE(flags.arm_state); // Servos retain navigation control throughout the glide.
}

static void test_failsafe_glide_keeps_motor_off_after_gps_recovery() {
  fly_north_of_home(200.0f);
  lose_link();
  step(2000);
  TEST_ASSERT_TRUE(state.throttle > 0.0f);
  state.heading_confidence = 0.0f;
  state.gps_lock = false;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, wing_nav_command.throttle);
  step(2000);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  set_gps(200.0f, 0.0f, -15.0f, 0.0f);
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  // Pilot recovery can select a powered return again.
  flags.failsafe_signal_lost = 0;
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > 0.0f);
  TEST_ASSERT_TRUE(wing_nav_command.throttle < 0.01f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.throttle);
}

static void test_failsafe_brief_gps_dip_resumes_return() {
  fly_north_of_home(200.0f);
  state.gps_lock = false;
  lose_link();
  step(400);
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  set_gps(200.0f, 0.0f, 15.0f, 0.0f);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.throttle);
}

// Cruise latches the eastbound ground track from fly() once the sticks have
// stayed centered for the lock delay.
static void cruise_locked() {
  fly(1U << AUX_CRUISE);
  step(600);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE, state.wing_nav_state);
}

static void test_cruise_locks_course_after_centered_sticks() {
  fly(1U << AUX_CRUISE);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_STEER, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("CRUISE", control_flight_mode_name());
  step(400);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_STEER, state.wing_nav_state);
  step(200);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("CRUISE", control_flight_mode_name());

  // On the track: wings level, and navigation owns the throttle.
  set_gps(0.0f, 30.0f, 0.0f, 15.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, roll_command_deg());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.throttle);
}

static void test_cruise_steers_back_onto_track() {
  cruise_locked();
  // Displaced north, left of the eastbound track, still flying east.
  set_gps(40.0f, 30.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_TRUE(roll_command_deg() > 1.0f);
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
}

static void test_cruise_roll_input_steers_and_relocks() {
  cruise_locked();
  state.rx_filtered.roll = 0.5f;
  // Control applies the release on the pass after navigation sees it.
  step(20);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_STEER, state.wing_nav_state);
  // The pilot's bank, as in Level.
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);

  // Release on a northbound track; it locks after the delay.
  state.rx_filtered.roll = 0.0f;
  set_gps(0.0f, 30.0f, 15.0f, 0.0f);
  step(400);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_STEER, state.wing_nav_state);
  step(200);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE, state.wing_nav_state);

  // Right of the new northbound track: turn left back onto it.
  set_gps(30.0f, 60.0f, 15.0f, 0.0f);
  step(1000);
  TEST_ASSERT_TRUE(roll_command_deg() < -1.0f);
}

static void test_cruise_yaw_input_releases_course() {
  cruise_locked();
  state.rx_filtered.yaw = 0.5f;
  step(20);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_STEER, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.setpoint.yaw > 0.0f);
}

static void test_cruise_releases_course_without_gps() {
  cruise_locked();
  state.gps_lock = false;
  state.rx_filtered.roll = 0.5f;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE_NO_GPS, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("CRUISE", control_flight_mode_name());
  // The pilot banks as while steering.
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
  state.rx_filtered.roll = 0.0f;

  set_gps(0.0f, 30.0f, 0.0f, 15.0f);
  step(600);
  TEST_ASSERT_EQUAL(WING_NAV_CRUISE, state.wing_nav_state);
}

static void cruise_with_baro(float altitude) {
  fly(0);
  state.baro_valid = true;
  state.altitude = altitude;
  state.baro_vertical_speed = 0.0f;
  state.aux_active |= 1U << AUX_CRUISE;
  step(1000);
}

static void test_cruise_pitch_stick_climbs_and_latches_altitude() {
  cruise_with_baro(50.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);

  // Pulling commands a climb, with throttle added for it.
  state.rx_filtered.pitch = -1.0f;
  step(1000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -2.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > profile.navigation.cruise_throttle);

  // Release latches the altitude reached, so higher is now above the target.
  state.rx_filtered.pitch = 0.0f;
  step();
  state.altitude = 55.0f;
  step(3000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch > 0.0f);
}

static void test_cruise_pitch_stick_sets_climb_angle_without_baro() {
  fly(1U << AUX_CRUISE);
  state.rx_filtered.pitch = -0.5f;
  step(1000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -10.0f, wing_nav_command.pitch * RADTODEG);
  TEST_ASSERT_TRUE(state.throttle > profile.navigation.cruise_throttle);
}

static void test_cruise_yields_to_loiter() {
  cruise_locked();
  state.aux_active |= 1U << AUX_LOITER;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
}

// Waypoints 300 m north, then 300 m east of that, at 50 m.
static void mission_route(mission_end_action_t end_action) {
  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  mission = {.count = 2, .end_action = end_action};
  const float route[2][2] = {{300.0f, 0.0f}, {300.0f, 300.0f}};
  for (unsigned i = 0; i < 2; i++) {
    mission.waypoints[i] = {
        .coord = {.lon = ORIGIN.lon + (int32_t)lrintf(route[i][1] / lon_scale * 1e7f),
                  .lat = ORIGIN.lat + (int32_t)lrintf(route[i][0] / METERS_PER_DEGREE_LAT * 1e7f)},
        .altitude = 50.0f,
        .speed = 0.0f,
    };
  }
}

// Move in fixes the GPS position-jump check accepts.
static void fly_track(float north0, float east0, float north1, float east1) {
  const float length = hypotf(north1 - north0, east1 - east0);
  const float vel_north = 15.0f * (north1 - north0) / length;
  const float vel_east = 15.0f * (east1 - east0) / length;
  for (float d = 20.0f; d <= length; d += 20.0f) {
    set_gps(north0 + (north1 - north0) * d / length, east0 + (east1 - east0) * d / length, vel_north, vel_east);
    step(100);
  }
}

// A mission starts from a switch edge in flight.
static void fly_mission() {
  fly(0);
  state.aux_active |= 1U << AUX_MISSION;
  step();
}

static void test_mission_steers_to_waypoints_in_order_and_circles_last() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.mission_active);
  TEST_ASSERT_EQUAL_STRING("MISSION", control_flight_mode_name());
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 300.0f, state.nav_target_distance);
  // Flying east with the waypoint to the north: turn left.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -profile.navigation.max_bank_angle, roll_command_deg());

  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, state.nav_target_bearing);
  step(2000);
  // Flying north with the next waypoint to the east: turn right.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, profile.navigation.max_bank_angle, roll_command_deg());

  fly_track(300.0f, 0.0f, 300.0f, 300.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.mission_active);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  // The circle passes through the arrival point; hold the last waypoint indefinitely.
  step(5000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);

  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);

  // A completed mission starts over.
  state.aux_active |= 1U << AUX_MISSION;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);

  // So does an interrupted one after a new arming.
  fly_track(300.0f, 300.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  state.aux_active = 0;
  state.rx_filtered.throttle = 0.0f;
  step();
  TEST_ASSERT_FALSE(flags.arm_state);
  fly_mission();
  step();
  TEST_ASSERT_TRUE(state.mission_active);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
}

static void test_mission_advances_past_a_missed_waypoint() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  step(100);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);

  // Blown 150 m east of the northbound leg, short of the waypoint and
  // outside the arrival radius: keep going.
  fly_track(0.0f, 0.0f, 250.0f, 150.0f);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);

  // Past the waypoint: an orbit would never enter the arrival radius, so
  // passing it counts as arrival.
  fly_track(250.0f, 150.0f, 310.0f, 150.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
}

static void test_mission_end_returns_home() {
  mission_route(MISSION_END_RTH);
  fly_mission();
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  fly_track(300.0f, 0.0f, 300.0f, 300.0f);
  step(100);
  TEST_ASSERT_FALSE(state.mission_active);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, state.home_distance, state.nav_target_distance);
}

static void test_mission_without_route_loiters_in_place() {
  mission = {};
  fly_mission();
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);

  // A route planned elsewhere is not flown either.
  mission_route(MISSION_END_HOLD);
  mission.waypoints[0].coord.lat += (int32_t)lrintf(5000.0f / METERS_PER_DEGREE_LAT * 1e7f);
  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  state.aux_active |= 1U << AUX_MISSION;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);
}

static void test_mission_yields_to_rth_and_failsafe() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  // Past the first waypoint and beyond the loiter radius, so a return flies toward home.
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  fly_track(300.0f, 0.0f, 300.0f, 100.0f);
  TEST_ASSERT_TRUE(state.mission_active);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  TEST_ASSERT_FALSE(state.mission_active);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);

  // Releasing RTH resumes toward the waypoint not yet reached.
  state.aux_active &= ~(1U << AUX_RETURN_TO_HOME);
  step(100);
  TEST_ASSERT_TRUE(state.mission_active);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);

  lose_link();
  step(2000);
  TEST_ASSERT_TRUE(state.wing_nav_failsafe);
  TEST_ASSERT_FALSE(state.mission_active);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
}

static void test_mission_needs_switch_edge_after_arming() {
  mission_route(MISSION_END_HOLD);
  // A switch left on through arming does not fly the stored route.
  fly(1U << AUX_MISSION);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);

  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  state.aux_active |= 1U << AUX_MISSION;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.mission_active);
}

static void test_mission_starts_once_gps_recovers() {
  mission_route(MISSION_END_HOLD);
  fly(0);
  state.gps_lock = false;
  state.aux_active |= 1U << AUX_MISSION;
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);

  // No new switch edge is needed once the route can be flown.
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.mission_active);
}

static void test_mission_gps_loss_returns_at_rth_altitude() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  // At the waypoint altitude, so the mission holds it level.
  state.baro_valid = true;
  state.altitude = mission.waypoints[0].altitude;
  state.baro_vertical_speed = 0.0f;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  // Heading home on the IMU climbs by the return altitude, as RTH does.
  state.heading_confidence = 1.0f;
  state.heading = 90.0f;
  state.gps_lock = false;
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HEADING, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.mission_active);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);

  // GPS recovery resumes the route at the waypoint altitude.
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_TRUE(state.mission_active);
}

void run_wing_navigation_tests() {
  RUN_TEST(test_loiter_requires_switch_and_flight);
  RUN_TEST(test_loiter_enters_tangentially_on_turn_side);
  RUN_TEST(test_loiter_left_direction_banks_left);
  RUN_TEST(test_loiter_turns_back_when_flying_away);
  RUN_TEST(test_loiter_caps_configured_bank);
  RUN_TEST(test_loiter_banks_without_gps_and_recenters_on_recovery);
  RUN_TEST(test_loiter_runs_without_configured_gps);
  RUN_TEST(test_turn_acceleration_keeps_imu_attitude);
  RUN_TEST(test_loiter_yields_to_failsafe_and_disarm);
  RUN_TEST(test_loiter_waits_for_autolaunch);
  RUN_TEST(test_loiter_holds_entry_altitude);
  RUN_TEST(test_loiter_integral_trims_steady_sink);
  RUN_TEST(test_loiter_levels_pitch_when_baro_goes_stale);
  RUN_TEST(test_failsafe_drops_without_failsafe_rth);
  RUN_TEST(test_failsafe_before_throw_disarms_without_rth);
  RUN_TEST(test_failsafe_during_launch_delay_cannot_start_rth_motor);
  RUN_TEST(test_failsafe_after_launch_spinup_can_return);
  RUN_TEST(test_failsafe_rth_preserves_throttle_during_handoff);
  RUN_TEST(test_rth_turns_toward_home);
  RUN_TEST(test_rth_circles_home_on_arrival);
  RUN_TEST(test_rth_climbs_by_return_altitude);
  RUN_TEST(test_rth_retains_altitude_target_across_baro_outages);
  RUN_TEST(test_rth_latches_altitude_when_baro_first_becomes_valid);
  RUN_TEST(test_rth_without_home_loiters_in_place);
  RUN_TEST(test_rth_steers_imu_heading_for_ten_seconds_after_gps_loss);
  RUN_TEST(test_rth_banks_at_once_without_heading_confidence);
  RUN_TEST(test_failsafe_rth_keeps_outputs_and_returns_control);
  RUN_TEST(test_failsafe_rth_continues_on_switch_after_recovery);
  RUN_TEST(test_failsafe_without_gps_descends_in_circle);
  RUN_TEST(test_failsafe_glide_keeps_motor_off_after_gps_recovery);
  RUN_TEST(test_failsafe_brief_gps_dip_resumes_return);
  RUN_TEST(test_cruise_locks_course_after_centered_sticks);
  RUN_TEST(test_cruise_steers_back_onto_track);
  RUN_TEST(test_cruise_roll_input_steers_and_relocks);
  RUN_TEST(test_cruise_yaw_input_releases_course);
  RUN_TEST(test_cruise_releases_course_without_gps);
  RUN_TEST(test_cruise_pitch_stick_climbs_and_latches_altitude);
  RUN_TEST(test_cruise_pitch_stick_sets_climb_angle_without_baro);
  RUN_TEST(test_cruise_yields_to_loiter);
  RUN_TEST(test_mission_steers_to_waypoints_in_order_and_circles_last);
  RUN_TEST(test_mission_advances_past_a_missed_waypoint);
  RUN_TEST(test_mission_end_returns_home);
  RUN_TEST(test_mission_without_route_loiters_in_place);
  RUN_TEST(test_mission_yields_to_rth_and_failsafe);
  RUN_TEST(test_mission_needs_switch_edge_after_arming);
  RUN_TEST(test_mission_starts_once_gps_recovers);
  RUN_TEST(test_mission_gps_loss_returns_at_rth_altitude);
}
