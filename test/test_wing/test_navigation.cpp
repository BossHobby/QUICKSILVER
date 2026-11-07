#include <math.h>
#include <string.h>
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
#include "io/quic_rt.h"
#include "util/util.h"

extern void imu_test_attitude_init();
extern void nav_test_set_vertical(float altitude, float climb);

// Script GPS fixes and run Flight's control/navigation order. This checks
// guidance direction and transitions, not airframe dynamics: the scripted
// aircraft flies straight at constant velocity, or around a scripted circle,
// whatever navigation commands. The receiver reports it as measured
// GPS_LAG_STEPS ago, as the navigation estimate assumes.
static constexpr gps_coord_t ORIGIN = {.lon = 1142000000, .lat = 223000000};
// Moving the scripted aircraft while GPS is held flies there at the scripted
// speed, no slower than this, so the estimate sees flight rather than a GPS
// jump.
static constexpr float TRAVEL_SPEED_MIN = 15.0f; // m/s
static constexpr uint32_t GPS_LAG_STEPS = 12;  // 10 ms steps, the estimate's 120 ms receiver lag

// The scripted aircraft, meters from ORIGIN and m/s, and what the receiver
// measured over the last GPS_LAG_STEPS steps.
static struct {
  vec2_t position;
  vec2_t velocity;
} scripted, scripted_history[GPS_LAG_STEPS];
static uint32_t scripted_index;
// Circling, scripted.position turns about this center, meters from ORIGIN.
static struct {
  bool active;
  vec2_t center;
} scripted_circle;

// Report the measurement GPS_LAG_STEPS old and record the current one.
static void publish_gps() {
  const auto reported = scripted_history[scripted_index];
  scripted_history[scripted_index] = scripted;
  scripted_index = (scripted_index + 1) % GPS_LAG_STEPS;

  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  state.gps_coord.lat = ORIGIN.lat + (int32_t)lrintf(reported.position.north / METERS_PER_DEGREE_LAT * 1e7f);
  state.gps_coord.lon = ORIGIN.lon + (int32_t)lrintf(reported.position.east / lon_scale * 1e7f);
  state.gps_velocity.north = reported.velocity.north;
  state.gps_velocity.east = reported.velocity.east;
  state.gps_speed = vec2_magnitude(reported.velocity);
  state.gps_last_update_ms = time_millis();
}

static void step(uint32_t milliseconds = 10);

static void set_gps(float north, float east, float vel_north, float vel_east) {
  const vec2_t target = {{north, east}};
  scripted_circle.active = false;
  if (state.gps_health.lock) {
    // A held fix moves only as fast as an aircraft flies.
    const vec2_t delta = target - scripted.position;
    const float distance = vec2_magnitude(delta);
    if (distance > 0.5f) {
      const float speed = MAX(hypotf(vel_north, vel_east), TRAVEL_SPEED_MIN);
      scripted.velocity = delta * (speed / distance);
      step((uint32_t)(distance / speed * 1000.0f) / 10 * 10);
    }
    scripted.position = target;
    scripted.velocity = {{vel_north, vel_east}};
  } else {
    // A new fix after none: the receiver has measured it all along.
    scripted.position = target;
    scripted.velocity = {{vel_north, vel_east}};
    for (auto &entry : scripted_history)
      entry = scripted;
  }
  state.gps_health.lock = true;
  state.gps_sats = 12; // a good fix, as the plant reports
  state.gps_horizontal_accuracy = 1.0f;
  publish_gps();
}

// Fly on around the circle through the current position about center, in the
// current direction at the current speed.
static void fly_circle(float center_north, float center_east) {
  scripted_circle = {.active = true, .center = {{center_north, center_east}}};
}

// Fly on around a circle of radius on the loiter side of the track, as the
// loiter places it at entry.
static void fly_loiter_circle(float radius) {
  const vec2_t v = scripted.velocity;
  const float side = profile.navigation.loiter_direction == NAV_LOITER_LEFT ? -1.0f : 1.0f;
  const vec2_t right = vec2_t{{-v.east, v.north}} * (side * radius / vec2_magnitude(v));
  const vec2_t center = scripted.position + right;
  fly_circle(center.north, center.east);
}

// Meters and degrees true from the scripted aircraft to a point, meters from ORIGIN.
static float scripted_distance_to(float north, float east) {
  return vec2_magnitude(vec2_t{{north, east}} - scripted.position);
}
static float scripted_bearing_to(float north, float east) {
  const vec2_t delta = vec2_t{{north, east}} - scripted.position;
  return normalize_deg(atan2f(delta.east, delta.north) * RADTODEG);
}

// Advance the scripted aircraft by dt seconds.
static void move_scripted(float dt) {
  if (!scripted_circle.active) {
    scripted.position += scripted.velocity * dt;
    return;
  }
  // Turn the radius by the arc flown; positive cross means a right turn.
  const vec2_t radial = scripted.position - scripted_circle.center;
  const float radius = vec2_magnitude(radial);
  const float speed = vec2_magnitude(scripted.velocity);
  const float angle = copysignf(speed / radius * dt, vec2_cross(radial, scripted.velocity));
  const float c = cosf(angle), s = sinf(angle);
  const vec2_t turned = {{radial.north * c - radial.east * s, radial.north * s + radial.east * c}};
  scripted.position = scripted_circle.center + turned;
  // Tangent along the direction of travel.
  const vec2_t tangent = angle > 0.0f ? vec2_t{{-turned.east, turned.north}} : vec2_t{{turned.east, -turned.north}};
  scripted.velocity = tangent * (speed / radius);
}

// Scripted vertical estimate: meters above launch, NAN without a barometer,
// changing at scripted_climb m/s.
static float scripted_altitude = NAN;
static float scripted_climb = 0.0f;

static void step(uint32_t milliseconds) {
  for (uint32_t i = 0; i < milliseconds; i += 10) {
    time_test_advance_us(10000);
    if (state.gps_health.lock) {
      move_scripted(0.01f);
      publish_gps();
    }
    if (float_is_finite(scripted_altitude)) {
      // Unaccelerated flight, so the climb rate stays at scripted_climb.
      scripted_altitude += scripted_climb * 0.01f;
      state.accel_raw = state.GEstG;
      nav_test_set_vertical(scripted_altitude, scripted_climb);
    }
    control();
    nav_update();
  }
}

static void fly(uint32_t aux) {
  nav_init();
  scripted_altitude = NAN;
  scripted_climb = 0.0f;
  profile.serial.gps = SERIAL_PORT1;
  set_gps(0.0f, 0.0f, 0.0f, 15.0f); // East at 15 m/s.
  // Disarmed first, as after boot, so mission progress starts over.
  step();
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
  fly_loiter_circle(profile.navigation.loiter_radius);
  step(10000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  // On the circle placed at the entry point: bank for v^2/r and hold it.
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.throttle);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);
}

static void test_loiter_left_direction_banks_left() {
  profile.navigation.loiter_direction = NAV_LOITER_LEFT;
  fly(1U << AUX_LOITER);
  fly_loiter_circle(profile.navigation.loiter_radius);
  step(10000);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -expected, roll_command_deg());
  TEST_ASSERT_TRUE(state.setpoint.roll < 0.0f);
}

static void test_loiter_tight_circle_banks_for_v2_over_r() {
  // About 30 degrees, where a small-angle bank would be 3 degrees too steep.
  profile.navigation.loiter_radius = 40.0f;
  fly(1U << AUX_LOITER);
  fly_loiter_circle(profile.navigation.loiter_radius);
  step(10000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
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
  state.gps_health.lock = false;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, roll_command_deg());

  // The aircraft drifted; the circle is placed again from the new fix.
  set_gps(30.0f, 30.0f, 15.0f, 0.0f);
  fly_loiter_circle(profile.navigation.loiter_radius);
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
  state.gyro_delta_angle = state.gyro * state.looptime;
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
  flags.wing_launch_available = true;
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
  state.rx_frame_time_us = time_micros();
  flags.failsafe_signal_lost = 1;
  step(400);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_FALSE(flags.wing_nav_failsafe);
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
  state.rx_frame_time_us = time_micros();
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
  TEST_ASSERT_TRUE(flags.wing_launch_available);
  state.rx_filtered.throttle = 0.5f;
  step(100);
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_TRUE(state.wing_launch_state < WING_LAUNCH_DONE);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
}

static void fly_with_baro(float altitude) {
  fly(0);
  // Scripted after arming, so the arming re-zero does not apply.
  scripted_altitude = altitude;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_LOITER;
  step(1000);
}

static void test_loiter_holds_entry_altitude() {
  fly_with_baro(50.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, wing_nav_command.throttle);

  // Below the target: nose up, with throttle added for the climb.
  scripted_altitude = 45.0f;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.pitch >= -20.0f * DEGTORAD - 0.001f);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > profile.navigation.cruise_throttle);
  TEST_ASSERT_TRUE(state.setpoint.pitch < 0.0f);

  // Above the target: nose down, with throttle reduced for the dive.
  scripted_altitude = 55.0f;
  step(4000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch > 5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.pitch <= 15.0f * DEGTORAD + 0.001f);
  TEST_ASSERT_TRUE(wing_nav_command.throttle < profile.navigation.cruise_throttle);
}

static void test_loiter_integral_trims_steady_sink() {
  fly_with_baro(50.0f);
  // A steady sink at the target altitude, as in a bank without extra lift.
  scripted_climb = -1.0f;
  step(5000);
  const float trimmed = wing_nav_command.pitch;
  TEST_ASSERT_TRUE(trimmed < -3.0f * DEGTORAD);
  scripted_climb = 0.0f;
  step(1000);
  // The learned nose-up trim remains once the sink has stopped.
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -1.0f * DEGTORAD);
}

static void test_loiter_levels_pitch_when_baro_goes_stale() {
  fly_with_baro(50.0f);
  scripted_altitude = 40.0f;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < 0.0f);
  scripted_altitude = NAN;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, wing_nav_command.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, profile.navigation.cruise_throttle, wing_nav_command.throttle);

  // A fresh barometer latches the current altitude rather than chasing the old target.
  scripted_altitude = 40.0f;
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
  state.rx_frame_time_us = time_micros();
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
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
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
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
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
  fly_circle(0.0f, 0.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HOME, state.wing_nav_state);
  const float expected = atanf(15.0f * 15.0f / (9.80665f * profile.navigation.loiter_radius)) * RADTODEG;
  TEST_ASSERT_FLOAT_WITHIN(0.5f, expected, roll_command_deg());
}

static void test_rth_climbs_by_return_altitude() {
  fly(0);
  scripted_altitude = 20.0f;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
  TEST_ASSERT_TRUE(wing_nav_command.throttle > profile.navigation.cruise_throttle);
}

static void test_rth_retains_altitude_target_across_baro_outages() {
  fly(0);
  scripted_altitude = 20.0f;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step();
  scripted_altitude = 20.0f + profile.navigation.rth_altitude;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  scripted_altitude = NAN;
  step(1000);
  scripted_altitude = 20.0f + profile.navigation.rth_altitude;
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
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);
}

static void test_rth_latches_altitude_when_baro_first_becomes_valid() {
  fly(0);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(1000);
  scripted_altitude = 20.0f;
  step();
  scripted_altitude = 20.0f + profile.navigation.rth_altitude;
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
  state.gps_health.lock = false; // No fix on arming: no home.
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
  state.gps_health.lock = false;
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
  state.gps_health.lock = false;
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
}

static void test_failsafe_rth_keeps_outputs_and_returns_control() {
  fly_north_of_home(200.0f);
  lose_link();
  step(3000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("FS RTH", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, wing_nav_command.throttle, state.mixer_source[OUTPUT_SOURCE_THROTTLE]);

  flags.failsafe_signal_lost = 0;
  step(20);
  TEST_ASSERT_FALSE(flags.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("MANUAL", control_flight_mode_name());
}

static void test_failsafe_rth_continues_on_switch_after_recovery() {
  fly_north_of_home(200.0f);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  lose_link();
  step(2000);
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
  flags.failsafe_signal_lost = 0;
  step(20);
  TEST_ASSERT_FALSE(flags.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
}

static void test_failsafe_without_gps_descends_in_circle() {
  fly(0);
  scripted_altitude = 50.0f;
  scripted_climb = -4.0f; // A glide must not pitch up to chase a powered sink-rate target.
  state.gps_health.lock = false;
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
  scripted_altitude = 0.0f;
  scripted_climb = 0.0f;
  step(10000);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  scripted_altitude = NAN;
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
  state.gps_health.lock = false;
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
  state.gps_health.lock = false;
  lose_link();
  step(400);
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);

  set_gps(200.0f, 0.0f, 15.0f, 0.0f);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
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

  // Steered onto a northbound track, then released; it locks after the delay.
  set_gps(0.0f, 30.0f, 15.0f, 0.0f);
  state.rx_filtered.roll = 0.0f;
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
  state.gps_health.lock = false;
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
  scripted_altitude = altitude;
  scripted_climb = 0.0f;
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
  scripted_altitude = 55.0f;
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
        .altitude = 50,
        .speed = 0,
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
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
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
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  // The circle passes through the arrival point; hold the last waypoint indefinitely.
  step(5000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);

  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

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
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
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

static void test_mission_holds_the_leg_line() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  // Drifted 100 m east of the northbound leg, already flying straight at the
  // waypoint: steer back onto the line rather than curve toward the waypoint.
  fly_track(0.0f, 0.0f, 100.0f, 100.0f);
  set_gps(100.0f, 100.0f, 15.0f * 200.0f / hypotf(200.0f, 100.0f), -15.0f * 100.0f / hypotf(200.0f, 100.0f));
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_TRUE(roll_command_deg() < -5.0f);

  // Back on the line and tracking it: fly straight.
  fly_track(100.0f, 100.0f, 150.0f, 0.0f);
  set_gps(150.0f, 0.0f, 15.0f, 0.0f);
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, roll_command_deg());
}

static void test_mission_turns_late_onto_a_shallow_leg() {
  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  mission_route(MISSION_END_HOLD);
  // Bend the second leg by under 6 degrees.
  mission.waypoints[1].coord = {
      .lon = ORIGIN.lon + (int32_t)lrintf(30.0f / lon_scale * 1e7f),
      .lat = ORIGIN.lat + (int32_t)lrintf(600.0f / METERS_PER_DEGREE_LAT * 1e7f),
  };
  fly_mission();
  // 20 m short is inside the full turn radius, but this turn needs a fraction of it.
  const float turn_radius = 15.0f * 15.0f / (9.80665f * tanf(profile.navigation.max_bank_angle * DEGTORAD));
  TEST_ASSERT_TRUE(turn_radius > 20.0f);
  fly_track(0.0f, 0.0f, 280.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  fly_track(280.0f, 0.0f, 320.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
}

static void test_mission_changes_altitude_over_the_leg() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  scripted_altitude = 20.0f;
  scripted_climb = 0.0f;
  step(1000);
  // At the start of the leg, 30 m below the waypoint: no climb yet.
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  // Near the end of the leg the target is close to the waypoint's altitude.
  fly_track(0.0f, 0.0f, 260.0f, 0.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
}

static void test_mission_climbs_in_place_before_the_first_leg() {
  mission_route(MISSION_END_HOLD);
  fly(0);
  scripted_altitude = 20.0f;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_MISSION;
  step(2000);
  // Far below the first waypoint, as a multirotor does: climb where it starts.
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);
  // Circling ahead on the turn side, not around the waypoint 300 m north.
  TEST_ASSERT_TRUE(roll_command_deg() > 5.0f);

  scripted_altitude = 48.0f;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);

  // A waypoint reached below its altitude is flown on from, not circled.
  scripted_altitude = 30.0f;
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
}

static void test_mission_circles_a_waypoint_for_its_hold_time() {
  mission_route(MISSION_END_HOLD);
  mission.waypoints[0].hold = 10;
  fly_mission();
  // North onto the waypoint; the hold runs from arrival at its radius.
  set_gps(0.0f, 0.0f, 15.0f, 0.0f);
  for (int i = 0; i < 300 && state.wing_nav_state != WING_NAV_MISSION_HOLD; i++)
    step(100);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_TRUE(flags.mission_engaged);
  fly_loiter_circle(profile.navigation.loiter_radius);
  step(9000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
}

static void test_mission_end_returns_home() {
  mission_route(MISSION_END_RTH);
  fly_mission();
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  fly_track(300.0f, 0.0f, 300.0f, 300.0f);
  step(100);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, state.home_distance, state.nav_target_distance);
}

static void test_mission_without_route_loiters_in_place() {
  mission = {};
  fly_mission();
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  // A route planned elsewhere is not flown either.
  mission_route(MISSION_END_HOLD);
  mission.waypoints[0].coord.lat += (int32_t)lrintf(5000.0f / METERS_PER_DEGREE_LAT * 1e7f);
  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  state.aux_active |= 1U << AUX_MISSION;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
}

static void test_mission_yields_to_rth_and_failsafe() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  // Past the first waypoint and beyond the loiter radius, so a return flies toward home.
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  fly_track(300.0f, 0.0f, 300.0f, 100.0f);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  state.aux_active |= 1U << AUX_RETURN_TO_HOME;
  step(100);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);

  // Releasing RTH resumes toward the waypoint not yet reached.
  state.aux_active &= ~(1U << AUX_RETURN_TO_HOME);
  step(100);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);

  // Without mission_on_failsafe, failsafe RTH takes the link loss over.
  profile.navigation.mission_on_failsafe = false;
  lose_link();
  step(2000);
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
}

static void test_mission_continues_through_link_loss() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);

  lose_link();
  step(2000);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_TRUE(flags.failsafe);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_FALSE(flags.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("FS MISSION", control_flight_mode_name());

  // It finishes and circles the last waypoint for as long as the link is lost.
  fly_track(300.0f, 0.0f, 300.0f, 300.0f);
  step(30000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_TRUE(flags.arm_state);

  // The link returns with the mission switch off. The switches stay locked
  // until a stick moves, which only unlocks them.
  flags.failsafe_signal_lost = 0;
  state.aux_active &= ~(1U << AUX_MISSION);
  step(1000);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  state.rx.roll = 0.5f;
  step(20);
  TEST_ASSERT_NOT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  state.rx.roll = 0.0f;
}

static void test_mission_link_recovery_keeps_the_mission_switch() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  fly_track(0.0f, 0.0f, 200.0f, 0.0f);
  lose_link();
  step(2000);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);

  // A switch left on keeps the mission flying once the sticks unlock it.
  flags.failsafe_signal_lost = 0;
  step(1000);
  state.rx.pitch = -0.5f;
  step(20);
  TEST_ASSERT_NOT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL_STRING("MISSION", control_flight_mode_name());
  state.rx.pitch = 0.0f;
}

static void test_mission_link_loss_without_gps_hands_over_to_failsafe() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  fly_track(0.0f, 0.0f, 200.0f, 0.0f);
  lose_link();
  step(2000);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);

  // The IMU-heading return still belongs to the mission. Once it times out
  // the route cannot be flown, and failsafe takes over without disarming.
  state.heading_confidence = 1.0f;
  state.heading = 90.0f;
  state.gps_health.lock = false;
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HEADING, state.wing_nav_state);
  TEST_ASSERT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  step(10000);
  TEST_ASSERT_NOT_EQUAL(FAILSAFE_OWNER_MISSION, state.failsafe_owner);
  TEST_ASSERT_TRUE(flags.wing_nav_failsafe);
  TEST_ASSERT_EQUAL(WING_NAV_DESCEND, state.wing_nav_state);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
}

static void test_mission_needs_switch_edge_after_arming() {
  mission_route(MISSION_END_HOLD);
  // A switch left on through arming does not fly the stored route.
  fly(1U << AUX_MISSION);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  state.aux_active |= 1U << AUX_MISSION;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
}

static void test_mission_starts_once_gps_recovers() {
  mission_route(MISSION_END_HOLD);
  fly(0);
  state.gps_health.lock = false;
  state.aux_active |= 1U << AUX_MISSION;
  step(100);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER_BANK, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  // No new switch edge is needed once the route can be flown.
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
}

static void test_mission_changes_altitude_evenly_along_the_leg() {
  mission_route(MISSION_END_HOLD);
  fly(0);
  scripted_altitude = 65.0f;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_MISSION;
  step();
  // A descent is flown along the leg rather than in place.
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  // Along the 300 m leg from 65 m to 50 m on its slope, well within the
  // wing's sink rate, past halfway.
  // The scripted climb does not follow the commands, so the climb angle
  // reads as the correction to the leg's sink rate.
  constexpr float sink = 15.0f / 300.0f * 15.0f; // m/s
  scripted_climb = -sink;
  fly_track(0.0f, 0.0f, 180.0f, 0.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  // On the slope at its sink rate: nothing to correct.
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);
  // Above it, it dives back toward it.
  scripted_altitude += 10.0f;
  step(2000);
  TEST_ASSERT_TRUE(wing_nav_command.pitch > 3.0f * DEGTORAD);
}

static void test_mission_gps_loss_returns_at_rth_altitude() {
  mission_route(MISSION_END_HOLD);
  fly(0);
  // Starting at the waypoint altitude, so the leg holds it level.
  scripted_altitude = mission.waypoints[0].altitude;
  scripted_climb = 0.0f;
  state.aux_active |= 1U << AUX_MISSION;
  step(2000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * DEGTORAD, 0.0f, wing_nav_command.pitch);

  // Heading home on the IMU climbs by the return altitude, as RTH does.
  state.heading_confidence = 1.0f;
  state.heading = 90.0f;
  state.gps_health.lock = false;
  step(2000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HEADING, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  TEST_ASSERT_TRUE(wing_nav_command.pitch < -5.0f * DEGTORAD);

  // GPS recovery resumes the route at the waypoint altitude.
  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
}

// Ground station commands over the QUIC_RT link, delivered to Flight by nav_update().
static void send_command(nav_command_type_t type, const void *args = nullptr, uint8_t size = 0) {
  static uint8_t id = 0;
  uint8_t frame[5 + NAV_COMMAND_ITEMS_MAX * sizeof(quic_rt_mission_item_t)] = {QUIC_RT_VERSION, QUIC_RT_COMMAND, (uint8_t)type, ++id};
  if (size)
    memcpy(frame + 4, args, size);
  quic_rt_receive(0x10, frame, 4 + size);
  step();
}

static void send_goto(float north, float east, float altitude) {
  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  const quic_rt_goto_t msg = {
      .lat = ORIGIN.lat + (int32_t)lrintf(north / METERS_PER_DEGREE_LAT * 1e7f),
      .lon = ORIGIN.lon + (int32_t)lrintf(east / lon_scale * 1e7f),
      .altitude = (int16_t)lroundf(altitude),
  };
  send_command(NAV_COMMAND_GOTO, &msg, sizeof(msg));
}

static quic_rt_status_t link_status() {
  step(200);
  uint8_t payload[QUIC_RT_STATUS_SIZE];
  uint8_t destination;
  TEST_ASSERT_EQUAL_UINT8(QUIC_RT_STATUS_SIZE, quic_rt_status(payload, &destination));
  TEST_ASSERT_EQUAL_HEX8(0x10, destination);
  quic_rt_status_t status;
  memcpy(&status, payload + 2, sizeof(status));
  return status;
}

static uint8_t link_result() {
  return link_status().result_index >> QUIC_RT_RESULT_SHIFT;
}

static void test_guided_flies_to_target_and_circles_it() {
  mission = {};
  fly(0);
  // Without the mission switch the ground station has no authority.
  send_goto(300.0f, 0.0f, 50.0f);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_NO_AUTHORITY, link_result());
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);

  // The switch grants it, even without a stored mission to fly.
  state.aux_active |= 1U << AUX_MISSION;
  step();
  TEST_ASSERT_TRUE(flags.guided_ready);
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  send_goto(300.0f, 0.0f, 50.0f);
  const quic_rt_status_t status = link_status();
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, status.result_index >> QUIC_RT_RESULT_SHIFT);
  TEST_ASSERT_TRUE(status.flags & QUIC_RT_FLAG_NAVIGATING);
  TEST_ASSERT_EQUAL(NAV_TARGET_GUIDED, (status.flags >> QUIC_RT_FLAG_NAV_TARGET_SHIFT) & 0x3);
  TEST_ASSERT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_EQUAL_STRING("GUIDED", control_flight_mode_name());
  // Flying east with the target to the north: turn left.
  step(1000);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, scripted_bearing_to(300.0f, 0.0f), state.nav_target_bearing);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, -profile.navigation.max_bank_angle, roll_command_deg());

  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);

  // HOLD circles where the aircraft is now, even while already circling.
  fly_track(300.0f, 0.0f, 300.0f, 200.0f);
  send_command(NAV_COMMAND_HOLD);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, state.nav_target_distance);

  // Switching the mission off hands control back and forgets the target.
  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
  TEST_ASSERT_FALSE(state.nav_target == NAV_TARGET_GUIDED || flags.guided_ready);
  state.aux_active |= 1U << AUX_MISSION;
  step();
  TEST_ASSERT_EQUAL(WING_NAV_LOITER, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);
}

static void test_guided_resumes_the_mission_and_returns_home() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);

  send_goto(0.0f, 0.0f, 50.0f);
  TEST_ASSERT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  // The mission continues toward the waypoint it was flying to.
  const quic_rt_mission_t resume = {.index = NAV_COMMAND_MISSION_RESUME};
  send_command(NAV_COMMAND_MISSION, &resume, sizeof(resume));
  step();
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  const quic_rt_mission_t missing = {.index = 2};
  send_command(NAV_COMMAND_MISSION, &missing, sizeof(missing));
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_NO_MISSION, link_result());

  // RTH flies home; the mission switch still ends it.
  fly_track(300.0f, 0.0f, 300.0f, 100.0f);
  send_command(NAV_COMMAND_RTH);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_RETURN, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_HOME, state.nav_target);
  TEST_ASSERT_EQUAL_STRING("RTH", control_flight_mode_name());
  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  TEST_ASSERT_EQUAL(WING_NAV_INACTIVE, state.wing_nav_state);
}

static mission_waypoint_t route_point(float north, float east) {
  const float lon_scale = METERS_PER_DEGREE_LAT * cosf(ORIGIN.lat * 1e-7f * DEGTORAD);
  return {
      .coord = {.lon = ORIGIN.lon + (int32_t)lrintf(east / lon_scale * 1e7f),
                .lat = ORIGIN.lat + (int32_t)lrintf(north / METERS_PER_DEGREE_LAT * 1e7f)},
      .altitude = 50,
      .speed = 0,
  };
}

static void send_mission(const mission_t &route) {
  for (uint8_t index = 0; index < route.count; index += NAV_COMMAND_ITEMS_MAX) {
    const uint8_t count = MIN(route.count - index, NAV_COMMAND_ITEMS_MAX);
    uint8_t args[1 + NAV_COMMAND_ITEMS_MAX * sizeof(quic_rt_mission_item_t)] = {index};
    for (uint8_t i = 0; i < count; i++) {
      const mission_waypoint_t &wp = route.waypoints[index + i];
      const quic_rt_mission_item_t item = {
          .lat = wp.coord.lat,
          .lon = wp.coord.lon,
          .altitude = wp.altitude,
          .speed = wp.speed,
          .type = wp.type,
          .hold = wp.hold,
      };
      memcpy(args + 1 + i * sizeof(item), &item, sizeof(item));
    }
    send_command(NAV_COMMAND_MISSION_ITEMS, args, 1 + count * sizeof(quic_rt_mission_item_t));
    TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  }
  const quic_rt_mission_commit_t commit = {.count = route.count, .end_action = route.end_action, .crc = mission_crc(&route)};
  send_command(NAV_COMMAND_MISSION_COMMIT, &commit, sizeof(commit));
}

// The route can change in flight, without the ground station taking over.
static void test_mission_replaced_in_flight_keeps_the_waypoint() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  step(1000);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);

  // Adding a waypoint after the active one keeps the leg.
  mission_t route = mission;
  route.count = 4;
  route.waypoints[2] = route_point(0.0f, 300.0f);
  route.waypoints[3] = route_point(0.0f, 600.0f);
  send_mission(route);
  quic_rt_status_t status = link_status();
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, status.result_index >> QUIC_RT_RESULT_SHIFT);
  TEST_ASSERT_EQUAL_UINT8(4, status.mission_count);
  TEST_ASSERT_EQUAL_HEX16(mission_crc(&route), status.mission_crc);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_GUIDED, state.nav_target);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, scripted_bearing_to(300.0f, 0.0f), state.nav_target_bearing);

  // Moving the active waypoint turns toward its new position.
  route.waypoints[0] = route_point(0.0f, 300.0f);
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, state.nav_target_bearing);

  // A route shortened past the active waypoint ends at its new last one.
  fly_track(0.0f, 0.0f, 0.0f, 300.0f);
  TEST_ASSERT_EQUAL_UINT8(1, state.mission_index);
  route.count = 1;
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  step(1000);
  TEST_ASSERT_EQUAL_UINT8(0, state.mission_index);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);

  // An empty route cannot replace the flown one.
  route.count = 0;
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_NO_MISSION, link_result());
  TEST_ASSERT_EQUAL_UINT8(1, mission.count);
}

// Once the mission switch is off nothing is flown, so any route may replace it.
static void test_mission_replaced_after_the_mission_ends() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  step(1000);
  state.aux_active &= ~(1U << AUX_MISSION);
  step();
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  mission_t route = mission;
  route.count = 1;
  route.waypoints[0] = route_point(5000.0f, 0.0f);
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  route.count = 0;
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  TEST_ASSERT_EQUAL_UINT8(0, mission.count);
}

// A GPS dropout returns on the IMU heading, but the route resumes without a
// new check once GPS recovers, so a replacement is checked at once.
static void test_mission_replaced_during_gps_loss_is_checked() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  step(1000);
  state.heading_confidence = 1.0f;
  state.heading = 90.0f;
  state.gps_health.lock = false;
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_RTH_HEADING, state.wing_nav_state);
  TEST_ASSERT_NOT_EQUAL(NAV_TARGET_MISSION, state.nav_target);

  mission_t route = mission;
  route.count = 0;
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_NO_MISSION, link_result());
  route.count = 2;
  route.waypoints[0] = route_point(3000.0f, 0.0f);
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_TOO_FAR, link_result());
  TEST_ASSERT_EQUAL_UINT8(2, mission.count);

  set_gps(0.0f, 0.0f, 0.0f, 15.0f);
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION, state.wing_nav_state);
  TEST_ASSERT_EQUAL(NAV_TARGET_MISSION, state.nav_target);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 300.0f, state.nav_target_distance);
}

// Moving the last waypoint of a completed route moves the circle with it.
static void test_mission_completed_circle_follows_a_moved_waypoint() {
  mission_route(MISSION_END_HOLD);
  fly_mission();
  fly_track(0.0f, 0.0f, 300.0f, 0.0f);
  fly_track(300.0f, 0.0f, 300.0f, 300.0f);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);

  mission_t route = mission;
  route.waypoints[1] = route_point(300.0f, 900.0f);
  send_mission(route);
  TEST_ASSERT_EQUAL_UINT8(NAV_RESULT_OK, link_result());
  step(1000);
  TEST_ASSERT_EQUAL(WING_NAV_MISSION_HOLD, state.wing_nav_state);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, scripted_distance_to(300.0f, 900.0f), state.nav_target_distance);
  // Flying east straight at the far circle needs no bank.
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 0.0f, roll_command_deg());
}

void run_wing_navigation_tests() {
  RUN_TEST(test_loiter_requires_switch_and_flight);
  RUN_TEST(test_loiter_enters_tangentially_on_turn_side);
  RUN_TEST(test_loiter_left_direction_banks_left);
  RUN_TEST(test_loiter_tight_circle_banks_for_v2_over_r);
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
  RUN_TEST(test_mission_holds_the_leg_line);
  RUN_TEST(test_mission_turns_late_onto_a_shallow_leg);
  RUN_TEST(test_mission_changes_altitude_over_the_leg);
  RUN_TEST(test_mission_climbs_in_place_before_the_first_leg);
  RUN_TEST(test_mission_circles_a_waypoint_for_its_hold_time);
  RUN_TEST(test_mission_end_returns_home);
  RUN_TEST(test_mission_without_route_loiters_in_place);
  RUN_TEST(test_mission_yields_to_rth_and_failsafe);
  RUN_TEST(test_mission_continues_through_link_loss);
  RUN_TEST(test_mission_link_recovery_keeps_the_mission_switch);
  RUN_TEST(test_mission_link_loss_without_gps_hands_over_to_failsafe);
  RUN_TEST(test_mission_needs_switch_edge_after_arming);
  RUN_TEST(test_mission_starts_once_gps_recovers);
  RUN_TEST(test_mission_changes_altitude_evenly_along_the_leg);
  RUN_TEST(test_mission_gps_loss_returns_at_rth_altitude);
  RUN_TEST(test_guided_flies_to_target_and_circles_it);
  RUN_TEST(test_guided_resumes_the_mission_and_returns_home);
  RUN_TEST(test_mission_replaced_in_flight_keeps_the_waypoint);
  RUN_TEST(test_mission_replaced_after_the_mission_ends);
  RUN_TEST(test_mission_replaced_during_gps_loss_is_checked);
  RUN_TEST(test_mission_completed_circle_follows_a_moved_waypoint);
}
