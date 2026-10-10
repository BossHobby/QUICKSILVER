#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "control/control.h"
#include "control/imu.h"
#include "control/navigation.h"

#include "rx/rx.h"
#include "util/util.h"
#include "control/multi/navigation.h"
#include "core/profile.h"
#include "driver/time.h"
#include "driver/baro/baro.h"
#include "mock_helpers.h"

extern void imu_test_attitude_init();
extern void imu_test_attitude_update();

void setUp() { time_test_reset(); }
void tearDown() {}

extern void nav_test_reset(void);
extern void nav_test_update_rth(void);
extern void nav_test_altitude_sample(float altitude, uint32_t now_ms);
extern void nav_test_altitude_control(float target, float rate, float dt);
extern void control_test_flight_mode(void);
extern float control_test_throttle_input(void);
extern void nav_test_update_horizontal_control(float dt);
extern void nav_test_update_gps_sanity(void);
extern bool nav_test_gps_sane(void);
extern void nav_test_set_home_valid(bool valid);
extern void nav_test_set_gps_sane(bool sane);
extern void nav_test_vertical_update(float dt);
extern void nav_test_set_hover_learned(float throttle);
extern void nav_test_update_hover(float dt);
static void set_altitude_source(bool valid, uint32_t update_ms) {
  state.baro_valid = valid;
  state.baro_last_update_ms = update_ms;
}

#define GPS_DEG_10M 899

// Feed the vertical estimator one navigation step of level flight.
static void sim_vertical(float accel_up, float dt) {
  state.GEstG = (vec3_t){{0, 0, 1}};
  state.accel_raw = (vec3_t){{0, 0, 1.0f + accel_up / 9.80665f}};
  nav_test_vertical_update(dt);
}

static void navigation_test_setup(void) {
  mock_hardware_reset_all();
  time_test_reset();
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));

  baro_init();
  nav_init();
  profile.serial.gps = SERIAL_PORT1;
  state.gps_lock = true;
  state.gps_sats = GPS_MIN_SATS_FOR_LOCK;
  state.heading = 0.0f;
  state.gps_heading = 0.0f;
  state.gps_speed = 0.0f;
  profile.rate.level_max_angle = 60.0f;
  profile.rate.profile = STICK_RATE_PROFILE_1;
  profile.rate.rates[STICK_RATE_PROFILE_1].mode = RATE_MODE_ACTUAL;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_CENTER_SENSITIVITY].roll = 200.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_CENTER_SENSITIVITY].pitch = 200.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_CENTER_SENSITIVITY].yaw = 200.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_MAX_RATE].roll = 300.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_MAX_RATE].pitch = 300.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_MAX_RATE].yaw = 300.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_EXPO].roll = 0.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_EXPO].pitch = 0.0f;
  profile.rate.rates[STICK_RATE_PROFILE_1].rate[ACTUAL_EXPO].yaw = 0.0f;
  profile.navigation.rth_cruise_speed = 8.0f;
  profile.rate.level_max_angle = 25.0f;
  profile.navigation.rth_throttle_min = 0.1f;
  profile.navigation.rth_throttle_hover = 0.5f;
  profile.navigation.rth_throttle_max = 0.75f;
  profile.navigation.rth_altitude = 10.0f;
  profile.navigation.rth_on_failsafe = true;
  state.GEstG.yaw = 1;
  state.accel_raw = (vec3_t){{0, 0, 1}};
  imu_test_attitude_init();
  state.heading_confidence = 0.5f; // Established GPS heading before requesting RTH.
  nav_test_reset();
}

void test_navigation_target_north_commands_forward_pitch(void) {
  navigation_test_setup();

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = GPS_DEG_10M;
  state.gps_home.lon = 0;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.pitch > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.roll);
}

void test_navigation_target_south_commands_backward_pitch(void) {
  navigation_test_setup();

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = -GPS_DEG_10M;
  state.gps_home.lon = 0;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.pitch < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.roll);
}

void test_navigation_target_east_commands_right_roll(void) {
  navigation_test_setup();

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = 0;
  state.gps_home.lon = GPS_DEG_10M;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.roll > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.pitch);
}

void test_navigation_heading_east_target_north_commands_left_roll(void) {
  navigation_test_setup();

  state.heading = 90.0f;
  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = GPS_DEG_10M;
  state.gps_home.lon = 0;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.roll < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.pitch);
}

void test_navigation_north_velocity_overshoot_commands_backward_pitch(void) {
  navigation_test_setup();

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = GPS_DEG_10M;
  state.gps_home.lon = 0;
  state.gps_vel_north = 12.0f;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.pitch < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.roll);
}

void test_navigation_east_velocity_overshoot_commands_left_roll(void) {
  navigation_test_setup();

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = 0;
  state.gps_home.lon = GPS_DEG_10M;
  state.gps_vel_east = 12.0f;

  nav_test_update_horizontal_control(0.1f);

  TEST_ASSERT_TRUE(state.rx_override.roll < 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, state.rx_override.pitch);
}

void test_navigation_command_respects_level_angle_limit(void) {
  const float limits[] = {15.0f, 25.0f, 60.0f};
  for (float limit : limits) {
    navigation_test_setup();
    profile.rate.level_max_angle = limit;
    state.heading_confidence = 1;
    state.gps_home.lat = GPS_DEG_10M;
    state.gps_vel_north = -20; // Demand enough tilt to reach the limit.
    for (int i = 0; i < 200; i++) nav_test_update_horizontal_control(0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1, state.rx_override.pitch);
    state.heading_confidence = 0.5f;
    for (int i = 0; i < 200; i++) nav_test_update_horizontal_control(0.01f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.625f, state.rx_override.pitch);
  }
}

void test_navigation_rth_requires_recent_altitude_source(void) {
  navigation_test_setup();

  flags.arm_state = 1;
  state.home_distance = 20.0f;
  nav_test_set_home_valid(true);
  nav_test_set_gps_sane(true);
  set_altitude_source(false, 0);

  nav_rth_start();
  TEST_ASSERT_FALSE(state.rth_active);

  time_test_set_us(1000000);
  set_altitude_source(true, time_millis());

  nav_rth_start();
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_EQUAL_UINT8(RTH_STATE_CLIMB, state.rth_state);
  TEST_ASSERT_FALSE(state.rth_failsafe_active);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rth_yaw_rate);

  state.rth_yaw_rate = 0.25f;
  nav_rth_stop();
  TEST_ASSERT_FALSE(state.rth_active);
  TEST_ASSERT_FALSE(state.rth_failsafe_active);
  TEST_ASSERT_EQUAL_UINT8(RTH_STATE_INACTIVE, state.rth_state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.rth_yaw_rate);
}

void test_navigation_resets_gps_jump_baseline_after_outage(void) {
  navigation_test_setup();

  time_test_set_us(1000000);
  state.gps_horizontal_accuracy = 1.0f;
  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_last_update_ms = time_millis();

  nav_test_update_gps_sanity();
  TEST_ASSERT_TRUE(nav_test_gps_sane());

  time_test_advance_us((500U + 1U) * 1000U);
  nav_test_update_gps_sanity();
  TEST_ASSERT_FALSE(nav_test_gps_sane());

  state.gps_coord.lat = GPS_DEG_10M * 10;
  state.gps_coord.lon = 0;
  state.gps_last_update_ms = time_millis();

  nav_test_update_gps_sanity();
  TEST_ASSERT_TRUE(nav_test_gps_sane());
}

void test_navigation_yaw_rate_bypasses_pilot_curves(void) {
  navigation_test_setup();

  profile.navigation.rth_cruise_speed = 4.0f;
  state.gps_speed = 10.0f;
  state.heading = 0.0f;
  state.home_bearing = 90.0f;
  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = GPS_DEG_10M;
  state.gps_home.lon = 0;

  nav_test_update_horizontal_control(0.1f);

  const float expected = state.rth_yaw_rate;
  TEST_ASSERT_TRUE(expected > 0.0f);
  TEST_ASSERT_TRUE(expected <= 30.0f * DEGTORAD);
  nav_test_set_home_valid(true);
  nav_test_set_gps_sane(true);
  set_altitude_source(true, time_millis());
  state.home_distance = 20;
  nav_rth_start();
  nav_test_update_horizontal_control(0.1f);
  state.rx_filtered = state.rx_override;
  for (unsigned mode = RATE_MODE_SILVERWARE; mode <= RATE_MODE_ACTUAL; mode++) {
    profile.rate.rates[0].mode = static_cast<rate_modes_t>(mode);
    profile.rate.rates[0].rate[0].yaw = 100;
    profile.rate.rates[0].rate[1].yaw = 900;
    profile.rate.rates[0].rate[2].yaw = 0.8f;
    control_test_flight_mode();
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expected, state.setpoint.yaw);
  }
  nav_rth_stop();
  state.rx_filtered = (vec4_t){0};
  control_test_flight_mode();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, state.setpoint.yaw);
}


static void start_test_rth(void) {
  navigation_test_setup();
  time_test_set_us(1000000);
  flags.arm_state = 1;
  flags.rx_ready = 1;
  nav_test_set_home_valid(true);
  nav_test_set_gps_sane(true);
  nav_test_altitude_sample(0, time_millis());
  state.home_distance = 100;
  state.gps_coord.lat = 8990;
  state.throttle = 0.5f;
  nav_rth_start();
  TEST_ASSERT_TRUE(state.rth_active);
}

static void rth_step(uint32_t elapsed_us) {
  time_test_advance_us(elapsed_us);
  set_altitude_source(true, time_millis());
  nav_test_update_rth();
}

void test_navigation_climb_publishes_gps_acceleration(void) {
  start_test_rth();
  // CLIMB levels the craft and drag brakes it; the IMU needs that deceleration.
  float velocity = -12;
  for (int i = 0; i < 200; i++) {
    if ((i % 10) == 0) {
      state.gps_last_update_ms = time_millis();
      state.gps_vel_north = velocity;
    }
    velocity += 4.0f * 0.01f;
    rth_step(10000);
  }
  TEST_ASSERT_EQUAL_UINT8(RTH_STATE_CLIMB, state.rth_state);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, 4.0f, state.nav_accel_north);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, state.nav_accel_east);

  // Without a trusted heading the IMU would rotate it onto the wrong axis.
  state.heading_confidence = 0.5f * RTH_MIN_HEADING_CONFIDENCE;
  rth_step(10000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.nav_accel_north);
  state.heading_confidence = 1.0f;
  state.rth_state = RTH_STATE_ACQUIRE_HEADING;
  state.gps_last_update_ms = time_millis();
  rth_step(10000);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.nav_accel_north);

  nav_rth_stop();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.nav_accel_north);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.nav_accel_east);
}

void test_navigation_throttle_bypasses_pilot_curves(void) {
  start_test_rth();
  state.rx_override.throttle = state.rx_filtered.throttle = 0.4f;
  profile.rate.throttle_expo = 0.9f;
  profile.rate.throttle_mid = 0.2f;
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, control_test_throttle_input());
  state.aux_active = 1U << AUX_IDLE_UP;
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, control_test_throttle_input());
  nav_rth_stop();
  // Handback blends from the navigation throttle, then follows pilot curves.
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f, control_test_throttle_input());
  time_test_advance_us(500000);
  TEST_ASSERT_TRUE(fabsf(control_test_throttle_input() - 0.4f) > 0.02f);
}

void test_navigation_vertical_rate_uses_sample_interval(void) {
  navigation_test_setup();
  nav_test_altitude_sample(0, 0);
  float height = 0;
  uint32_t stamp = 0;
  for (int i = 0; i < 200; i++) {
    const uint32_t step = i % 2 ? 40 : 60;
    stamp += step;
    height += 2.0f * step * 0.001f;
    nav_test_altitude_sample(height, stamp);
    const float velocity = state.baro_vertical_speed;
    for (int j = 0; j < 5; j++) {
      nav_test_altitude_control(state.altitude, 2, 0.01f);
      TEST_ASSERT_FLOAT_WITHIN(0.00001f, velocity, state.baro_vertical_speed);
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f, state.baro_vertical_speed);
}

void test_navigation_altitude_integral_unwinds_after_saturation(void) {
  start_test_rth();
  profile.navigation.rth_throttle_max = 0.55f;
  for (int i = 0; i < 1000; i++) {
    nav_test_altitude_control(100, 2, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.55f, state.rx_override.throttle);
  // Reverse the error to unwind trim, then allow the rate request to settle.
  for (int i = 0; i < 300; i++) {
    nav_test_altitude_control(-1, 2, 0.01f);
  }
  for (int i = 0; i < 300; i++) {
    nav_test_altitude_control(0, 2, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.5f, state.rx_override.throttle);
  profile.navigation.rth_throttle_max = 0.75f;
  state.attitude.pitch = 30 * DEGTORAD;
  for (int i = 0; i < 100; i++) {
    nav_test_altitude_control(0, 2, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f / cosf(30 * DEGTORAD), state.rx_override.throttle);
}

void test_navigation_descent_uses_configured_throttle_minimum(void) {
  start_test_rth();
  state.altitude = 50;
  for (int i = 0; i < 2000; i++) {
    nav_test_altitude_control(10, 2, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, state.rx_override.throttle);
}

void test_navigation_rth_accepts_unestablished_heading(void) {
  start_test_rth();
  nav_rth_stop();
  state.heading_confidence = 0;
  nav_rth_start();
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(flags.controls_override);
  nav_rth_stop();
  flags.failsafe_signal_lost = 1;
  nav_rth_start();
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  // Confidence loss in flight is handled by recovery, not an abrupt drop.
  state.heading_confidence = 0;
  rth_step(10000);
  TEST_ASSERT_TRUE(state.rth_active);
}

void test_navigation_altitude_recovers_with_wrong_hover_setting(void) {
  const float hover_values[] = {0.2f, 0.5f, 0.65f};
  for (const float actual_hover : hover_values) {
    start_test_rth();
    const float dt = 0.01f;
    nav_rth_stop();
    state.throttle = actual_hover;
    nav_rth_start();
    float height = 0;
    float velocity = 0;
    float max_height = 0;
    // Keep the flight's 0.5 configured hover and vary the actual thrust response.
    for (int i = 0; i < 6000; i++) {
      time_test_advance_us(10000);
      if (i % 5 == 0)
        nav_test_altitude_sample(height, time_millis());
      nav_test_altitude_control(10, 2, dt);
      const float accel = 9.81f * (state.rx_override.throttle / actual_hover - 1) - 0.3f * velocity;
      velocity += dt * accel;
      sim_vertical(accel, dt);
      height += velocity * dt;
      max_height = MAX(max_height, height);
      TEST_ASSERT_TRUE(state.rx_override.throttle >= profile.navigation.rth_throttle_min);
      TEST_ASSERT_TRUE(state.rx_override.throttle <= profile.navigation.rth_throttle_max);
      if (i > 5000) {
        TEST_ASSERT_FLOAT_WITHIN(0.2f, 0, velocity);
        TEST_ASSERT_FLOAT_WITHIN(0.02f, actual_hover, state.rx_override.throttle);
      }
    }
    TEST_ASSERT_TRUE(max_height < 15);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 10, height);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0, velocity);
    char diagnostic[256];
    snprintf(diagnostic, sizeof(diagnostic), "alt final h=%f v=%f throttle=%f", (double)height, (double)velocity, (double)state.rx_override.throttle);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01f, actual_hover, state.rx_override.throttle, diagnostic);
  }
}

void test_navigation_altitude_handles_delay_and_noise(void) {
  const float hover_values[] = {0.2f, 0.5f, 0.65f};
  for (const float hover : hover_values) {
    start_test_rth();
    nav_rth_stop();
    state.throttle = hover;
    nav_rth_start();
    float history[30] = {};
    float height = 0, velocity = 0, thrust = hover;
    float peak_height = 0, min_throttle = 1, max_throttle = 0;
    for (int i = 0; i < 8000; i++) {
      time_test_advance_us(10000);
      const float delayed_height = history[i % 30];
      history[i % 30] = height;
      if (i % 5 == 0) {
        const float noise = 0.15f * sinf(i * 0.01f * 5) + 0.05f * sinf(i * 0.01f * 19);
        nav_test_altitude_sample(delayed_height + noise, time_millis());
      }
      nav_test_altitude_control(10, 2, 0.01f);
      thrust += 0.01f / 0.1f * (state.rx_override.throttle - thrust);
      const float accel = 9.81f * (thrust / hover - 1) - 0.3f * velocity;
      velocity += 0.01f * accel;
      sim_vertical(accel, 0.01f);
      height += velocity * 0.01f;
      peak_height = MAX(peak_height, height);
      if (i > 7000) {
        min_throttle = MIN(min_throttle, state.rx_override.throttle);
        max_throttle = MAX(max_throttle, state.rx_override.throttle);
        TEST_ASSERT_FLOAT_WITHIN(0.8f, 10, height);
        TEST_ASSERT_FLOAT_WITHIN(0.5f, 0, velocity);
      }
    }
    TEST_ASSERT_TRUE(peak_height < 13);
    TEST_ASSERT_TRUE(max_throttle - min_throttle < 0.15f);
  }
}

void test_navigation_altitude_takeover_is_smooth(void) {
  start_test_rth();
  nav_rth_stop();
  state.throttle = 0.2f;
  nav_rth_start();
  nav_test_altitude_control(10, 2, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.2f, state.rx_override.throttle);
  nav_test_altitude_control(10, 2, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.2f, state.rx_override.throttle);
}

void test_navigation_pitch_command_matches_imu_and_angle_control(void) {
  navigation_test_setup();
  state.looptime = 0.001f;
  state.looptime_inverse = 1000;
  state.looptime_autodetect = 1000;
  profile.pid.small_angle.kp = profile.pid.big_angle.kp = 2;
  profile.pid.small_angle.kd = profile.pid.big_angle.kd = 0;
  state.accel_raw = (vec3_t){{0, 0, 1}};
  imu_init();
  flags.arm_state = 1;
  flags.controls_override = 1;
  state.rth_active = true;
  state.gps_home.lat = GPS_DEG_10M;
  nav_test_update_horizontal_control(0.1f);
  const float command = state.rx_override.pitch;
  const float target_pitch = command * profile.rate.level_max_angle * DEGTORAD;
  TEST_ASSERT_TRUE(target_pitch > 0);

  // Feed the real IMU the gravity and gyro trajectory of a nose-down pitch.
  // Angle control must drive toward the demand, then oppose an overshoot.
  for (int i = 0; i < 2000; i++) {
    const float pitch = target_pitch * 1.5f * i / 1999;
    state.accel_raw = (vec3_t){{0, sinf(pitch), cosf(pitch)}};
    state.gyro.pitch = target_pitch * 1.5f / 2;
    state.gyro_delta_angle.pitch = state.gyro.pitch * state.looptime;
    imu_calc();
    state.rx_filtered = state.rx_override;
    control_test_flight_mode();
    if (i == 0) TEST_ASSERT_TRUE(state.setpoint.pitch > 0);
    if (i == 1999) TEST_ASSERT_TRUE(state.setpoint.pitch < 0);
    TEST_ASSERT_FLOAT_WITHIN(0.2f * DEGTORAD, pitch, state.attitude.pitch);
  }
}

void test_navigation_integral_stays_earth_referenced_during_yaw(void) {
  navigation_test_setup();
  state.gps_coord.lat = state.gps_home.lat = 0;
  state.gps_vel_north = -0.2f;
  for (int i = 0; i < 1500; i++) nav_test_update_horizontal_control(0.01f);
  state.gps_vel_north = 0;
  for (int i = 0; i < 150; i++) nav_test_update_horizontal_control(0.01f);
  const float pitch = state.rx_override.pitch;
  TEST_ASSERT_TRUE(pitch > 0);
  state.heading = 90;
  for (int i = 0; i < 150; i++) nav_test_update_horizontal_control(0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, state.rx_override.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -pitch, state.rx_override.roll);
}

void test_navigation_repeated_gps_sample_keeps_commands_smooth(void) {
  navigation_test_setup();
  state.gps_last_update_ms = 100;
  nav_test_update_horizontal_control(0.01f);
  state.gps_vel_north = 0.2f;
  state.gps_last_update_ms = 200;
  nav_test_update_horizontal_control(0.01f);
  TEST_ASSERT_TRUE(state.rx_override.pitch < 0);
  for (int i = 0; i < 9; i++) {
    const float previous = state.rx_override.pitch;
    nav_test_update_horizontal_control(0.01f);
    TEST_ASSERT_TRUE(state.rx_override.pitch < 0);
    TEST_ASSERT_TRUE(fabsf(state.rx_override.pitch - previous) <= 0.02001f);
  }
}

void test_navigation_altitude_loss_holds_and_resumes(void) {
  start_test_rth();
  time_test_advance_us(501000);
  nav_test_update_rth();
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(flags.controls_override);
  TEST_ASSERT_EQUAL(RTH_STATE_CLIMB, state.rth_state);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  TEST_ASSERT_TRUE(state.rx_override.throttle >= profile.navigation.rth_throttle_min);
  // An outage inside the hold window resumes the return.
  for (int i = 0; i < 900; i++) {
    time_test_advance_us(10000);
    nav_test_update_rth();
  }
  TEST_ASSERT_TRUE(state.rth_active);
  set_altitude_source(true, time_millis());
  state.altitude = 10;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);

  // A longer outage gives up and is not retried.
  start_test_rth();
  set_altitude_source(false, 0);
  for (int i = 0; i < 990; i++) {
    time_test_advance_us(10000);
    nav_test_update_rth();
  }
  TEST_ASSERT_TRUE(state.rth_active);
  for (int i = 0; i < 20; i++) {
    time_test_advance_us(10000);
    nav_test_update_rth();
  }
  TEST_ASSERT_FALSE(state.rth_active);
  TEST_ASSERT_FALSE(flags.controls_override);
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
  set_altitude_source(true, time_millis());
  nav_rth_start();
  TEST_ASSERT_FALSE(state.rth_active);
}

void test_navigation_nonfinite_altitude_holds(void) {
  start_test_rth();
  nav_test_altitude_sample(NAN, time_millis());
  nav_test_update_rth();
  TEST_ASSERT_TRUE(state.rth_active);
  start_test_rth();
  nav_test_altitude_sample(INFINITY, time_millis());
  nav_test_update_rth();
  TEST_ASSERT_TRUE(state.rth_active);
}

void test_navigation_altitude_loss_holds_vertical_speed(void) {
  start_test_rth();
  set_altitude_source(false, 0);
  // Without altitude, trim is frozen and the demand is zero vertical speed.
  state.vertical_speed = -1.0f;
  float before = 0;
  for (int i = 0; i < 100; i++) {
    time_test_advance_us(10000);
    nav_test_update_rth();
    if (i == 0) before = state.rx_override.throttle;
  }
  TEST_ASSERT_TRUE(state.rx_override.throttle > 0.5f);
  state.vertical_speed = 1.0f;
  for (int i = 0; i < 100; i++) {
    time_test_advance_us(10000);
    nav_test_update_rth();
  }
  TEST_ASSERT_TRUE(state.rx_override.throttle < 0.5f);
  TEST_ASSERT_TRUE(before >= profile.navigation.rth_throttle_min);
}

void test_navigation_gps_loss_holds_latched_altitude(void) {
  start_test_rth();
  nav_test_set_gps_sane(false);
  rth_step(10000);
  state.altitude = -1;
  for (int i = 0; i < 50; i++) rth_step(10000);
  TEST_ASSERT_TRUE(state.rx_override.throttle > 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, state.rx_override.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0, state.rth_yaw_rate);
  for (int i = 0; i < 900; i++) rth_step(10000);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_EQUAL(RTH_STATE_CLIMB, state.rth_state);
  for (int i = 0; i < 100; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
}

void test_navigation_flapping_gps_does_not_extend_return(void) {
  start_test_rth();
  state.altitude = 10;
  rth_step(10000);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAVIGATE, state.rth_state);
  // Short outages without progress still use up the progress window.
  for (int cycle = 0; cycle < 20 && state.rth_state == RTH_STATE_NAVIGATE; cycle++) {
    nav_test_set_gps_sane(cycle % 2);
    for (int i = 0; i < 100; i++) rth_step(10000);
  }
  TEST_ASSERT_EQUAL(RTH_STATE_NAV_FAILED, state.rth_state);
}

void test_navigation_gps_recovery_resumes_original_climb(void) {
  start_test_rth();
  nav_test_set_gps_sane(false);
  for (int i = 0; i < 100; i++) rth_step(10000);
  nav_test_set_gps_sane(true);
  for (int i = 0; i < 50; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_CLIMB, state.rth_state);
  TEST_ASSERT_TRUE(state.rx_override.throttle > 0.5f);
  state.altitude = 10;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAVIGATE, state.rth_state);
}

void test_navigation_stalled_climb_and_return_abort(void) {
  start_test_rth();
  for (int i = 0; i < 1001; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
  start_test_rth();
  state.altitude = 10;
  rth_step(10000);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAVIGATE, state.rth_state);
  for (int i = 0; i < 1501; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAV_FAILED, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.roll);
  // Hold level so the link can return, then give up.
  for (int i = 0; i < 998; i++) rth_step(10000);
  TEST_ASSERT_TRUE(state.rth_active);
  for (int i = 0; i < 2; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
}

void test_navigation_hover_does_not_timeout(void) {
  start_test_rth();
  state.altitude = 10;
  state.home_distance = 0;
  state.gps_coord = state.gps_home;
  for (int i = 0; i < 12000; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_HOVER_HOME, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(flags.arm_state);
}

static void failsafe_step(uint32_t us) {
  time_test_advance_us(us);
  if (!flags.failsafe_signal_lost) state.last_frame_time_us = time_micros();
  control_failsafe_update();
}

void test_navigation_failsafe_rth_waits_for_stable_link_and_sticks(void) {
  start_test_rth();
  flags.failsafe_signal_lost = 1;
  state.last_frame_time_us = time_micros();
  for (int i = 0; i < 2000; i++) failsafe_step(10000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);

  // Link back, sticks centered: RTH keeps control.
  flags.failsafe_signal_lost = 0;
  state.rx = (vec4_t){0};
  for (int i = 0; i < 100; i++) failsafe_step(10000);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  TEST_ASSERT_TRUE(flags.failsafe);

  // Sticks move, but the link drops again before it has been stable.
  state.rx.pitch = 0.5f;
  flags.failsafe_signal_lost = 1;
  failsafe_step(10000);
  flags.failsafe_signal_lost = 0;
  for (int i = 0; i < 40; i++) failsafe_step(10000);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  TEST_ASSERT_TRUE(flags.failsafe);

  // Stable for the recovery time with stick input: hand back.
  for (int i = 0; i < 11; i++) failsafe_step(10000);
  TEST_ASSERT_FALSE(state.rth_failsafe_active);
  TEST_ASSERT_FALSE(flags.failsafe);
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_navigation_flapping_link_does_not_interrupt_rth(void) {
  start_test_rth();
  state.rx.roll = 1.0f; // pilot input must not matter while the link is unstable
  flags.failsafe_signal_lost = 1;
  state.last_frame_time_us = time_micros();
  failsafe_step(10000);
  for (int cycle = 0; cycle < 50; cycle++) {
    flags.failsafe_signal_lost = cycle % 2;
    for (int i = 0; i < 20; i++) {
      failsafe_step(10000);
      TEST_ASSERT_TRUE(state.rth_failsafe_active);
      TEST_ASSERT_TRUE(flags.failsafe);
      TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);
    }
  }
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(flags.arm_state);
}

void test_navigation_failsafe_rth_gives_up_with_stage2(void) {
  start_test_rth();
  flags.failsafe_signal_lost = 1;
  state.last_frame_time_us = time_micros();
  failsafe_step(10000);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  nav_test_set_gps_sane(false);
  for (int i = 0; i < 1001 && state.rth_active; i++) {
    TEST_ASSERT_TRUE(flags.arm_state);
    rth_step(10000);
    control_failsafe_update();
  }
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
}

void test_navigation_failsafe_rth_disarms_after_crash(void) {
  start_test_rth();
  flags.failsafe_signal_lost = 1;
  state.last_frame_time_us = time_micros();
  failsafe_step(10000);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);

  // A brief upset while flying is not a crash.
  state.attitude.roll = 100 * DEGTORAD;
  for (int i = 0; i < 90; i++) failsafe_step(10000);
  state.attitude.roll = 0;
  failsafe_step(10000);
  state.attitude.roll = 100 * DEGTORAD;
  for (int i = 0; i < 90; i++) failsafe_step(10000);
  TEST_ASSERT_TRUE(flags.arm_state);
  TEST_ASSERT_FALSE(flags.failsafe_outputs_blocked);

  // Held beyond the tilt limit: the vehicle is down, stop the motors.
  for (int i = 0; i < 11; i++) failsafe_step(10000);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_TRUE(flags.failsafe_outputs_blocked);
  TEST_ASSERT_FALSE(state.rth_failsafe_active);
  for (int i = 0; i < 100; i++) failsafe_step(10000);
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
}

static void setup_failsafe_flight(float pilot_throttle) {
  navigation_test_setup();
  time_test_set_us(1000000);
  flags.arm_state = 1;
  flags.rx_ready = 1;
  flags.in_air = 1;
  state.rx_filtered.throttle = 0.5f;
  state.throttle = pilot_throttle;
  state.last_frame_time_us = time_micros();
  control_failsafe_update();
}

void test_navigation_failsafe_holds_pilot_throttle_instead_of_idle(void) {
  setup_failsafe_flight(0.45f);
  flags.failsafe_signal_lost = 1;
  for (int i = 0; i < 31; i++) failsafe_step(10000);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE1_GUARD, state.failsafe_phase);
  TEST_ASSERT_TRUE(state.failsafe_hold);
  TEST_ASSERT_TRUE(flags.controls_override);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.45f, control_test_throttle_input());
  TEST_ASSERT_EQUAL_STRING("FS HOLD", control_flight_mode_name());
  // Without RTH the existing stage 2 procedure still follows.
  for (int i = 0; i < 101; i++) failsafe_step(10000);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
  TEST_ASSERT_FALSE(state.failsafe_hold);
  TEST_ASSERT_FALSE(flags.arm_state);
}

void test_navigation_failsafe_on_ground_keeps_idle(void) {
  setup_failsafe_flight(0.45f);
  flags.in_air = 0;
  flags.failsafe_signal_lost = 1;
  for (int i = 0; i < 31; i++) failsafe_step(10000);
  TEST_ASSERT_FALSE(state.failsafe_hold);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.001f, control_test_throttle_input());

  // Throttle low for longer than the landing window also counts as landed.
  setup_failsafe_flight(0.45f);
  state.rx_filtered.throttle = 0;
  for (int i = 0; i < 1001; i++) failsafe_step(10000);
  flags.failsafe_signal_lost = 1;
  for (int i = 0; i < 31; i++) failsafe_step(10000);
  TEST_ASSERT_FALSE(state.failsafe_hold);
}

void test_navigation_failsafe_hold_levels_in_acro(void) {
  setup_failsafe_flight(0.45f);
  state.aux_active = 0; // acro
  flags.failsafe_signal_lost = 1;
  for (int i = 0; i < 31; i++) failsafe_step(10000);
  state.rx_filtered = state.rx_override;
  state.attitude.roll = 20 * DEGTORAD;
  state.GEstG = (vec3_t){{sinf(20 * DEGTORAD), 0, cosf(20 * DEGTORAD)}};
  control_test_flight_mode();
  // Angle mode drives the tilt back toward level instead of holding it.
  TEST_ASSERT_TRUE(fabsf(state.setpoint.roll) > 0.01f);
}

// Regression: failsafe RTH used to take over from idle throttle and fall ~20m.
static float simulate_failsafe_rth(bool learn_hover, float pilot_throttle, float forced_hover = 0.0f) {
  navigation_test_setup();
  time_test_set_us(1000000);
  const float hover = 0.45f;
  flags.rx_ready = 1;
  flags.arm_state = 1;
  flags.in_air = 1;
  state.gps_horizontal_accuracy = 1;
  float height = 0, velocity = 0, min_height = 0, pilot = hover;
  auto step = [&](int loops) {
    for (int i = 0; i < loops; i++) {
      time_test_advance_us(1000);
      state.gps_last_update_ms = time_millis();
      if (!flags.failsafe_signal_lost) state.last_frame_time_us = time_micros();
      if (i % 20 == 0) nav_test_altitude_sample(height, time_millis());
      control_failsafe_update();
      if (flags.controls_override) state.rx_filtered = state.rx_override;
      // Before stage 1 the pilot's (held) throttle flies the craft.
      state.throttle = flags.controls_override ? control_test_throttle_input() : pilot;
      const float accel = 9.81f * (state.throttle / hover - 1) - 0.3f * velocity;
      velocity += accel * 0.001f;
      height += velocity * 0.001f;
      min_height = MIN(min_height, height);
      state.GEstG = (vec3_t){{0, 0, 1}};
      state.accel_raw = (vec3_t){{0, 0, 1.0f + accel / 9.80665f}};
      nav_update();
    }
  };
  state.rx_filtered.throttle = 0.5f;
  step(10); // arm and capture home
  height = 5.0f; // clear of the ground, where hover learning applies
  state.gps_coord.lat = 2697; // 30m from home
  step(learn_hover ? 5000 : 500);
  if (learn_hover) TEST_ASSERT_FLOAT_WITHIN(0.01f, hover, state.hover_throttle);
  if (forced_hover > 0.0f) nav_test_set_hover_learned(forced_hover);
  // Optionally dive for a second before the link drops.
  pilot = pilot_throttle;
  step(pilot_throttle < hover ? 1000 : 0);
  const float loss_height = height;
  min_height = height;
  flags.failsafe_signal_lost = 1;
  step(8000);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  TEST_ASSERT_TRUE(flags.arm_state);
  return min_height - loss_height;
}

void test_navigation_failsafe_rth_takeover_does_not_drop(void) {
  const float unlearned = simulate_failsafe_rth(false, 0.45f);
  const float learned = simulate_failsafe_rth(true, 0.45f);
  const float diving = simulate_failsafe_rth(true, 0.3f);
  const float diving_unlearned = simulate_failsafe_rth(false, 0.3f);
  printf("failsafe takeover altitude loss: unlearned=%.2f learned=%.2f diving=%.2f diving_unlearned=%.2f\n",
      (double)unlearned, (double)learned, (double)diving, (double)diving_unlearned);
  TEST_ASSERT_TRUE(unlearned > -1.0f);
  TEST_ASSERT_TRUE(learned > -1.0f);
  TEST_ASSERT_TRUE(diving > -4.0f);
  TEST_ASSERT_TRUE(diving_unlearned > -6.0f);
}

void test_navigation_wrong_learned_hover_does_not_drop(void) {
  // Learned far too low (e.g. idling on the ground) or from a sagging battery:
  // the takeover estimate from the applied throttle overrides it.
  const float low = simulate_failsafe_rth(false, 0.45f, 0.12f);
  const float high = simulate_failsafe_rth(false, 0.45f, 0.70f);
  TEST_ASSERT_TRUE(low > -1.0f);
  TEST_ASSERT_TRUE(high > -1.0f);
}

void test_navigation_failsafe_on_ground_does_not_start_rth(void) {
  navigation_test_setup();
  time_test_set_us(1000000);
  flags.rx_ready = 1;
  flags.arm_state = 1;
  flags.in_air = 0;
  state.gps_horizontal_accuracy = 1;
  state.rx_filtered.throttle = 0;
  auto step = [&](int loops) {
    for (int i = 0; i < loops; i++) {
      time_test_advance_us(1000);
      state.gps_last_update_ms = time_millis();
      if (!flags.failsafe_signal_lost) state.last_frame_time_us = time_micros();
      if (i % 20 == 0) nav_test_altitude_sample(0, time_millis());
      control_failsafe_update();
      nav_update();
    }
  };
  step(100); // arm and capture home
  flags.failsafe_signal_lost = 1;
  for (int i = 0; i < 20; i++) {
    step(100);
    TEST_ASSERT_FALSE(state.rth_active);
  }
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL(FAILSAFE_PHASE_STAGE2_DROP, state.failsafe_phase);
}

void test_navigation_rth_starts_close_to_home(void) {
  start_test_rth();
  nav_rth_stop();
  state.home_distance = 1;
  nav_rth_start();
  TEST_ASSERT_TRUE(state.rth_active);
  state.gps_coord = state.gps_home;
  state.altitude = 10;
  rth_step(10000);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_HOVER_HOME, state.rth_state);
}

void test_navigation_hover_learning(void) {
  navigation_test_setup();
  flags.arm_state = 1;
  flags.in_air = 1;
  flags.on_ground = 0;
  set_altitude_source(true, time_millis());
  state.throttle = 0.4f;
  // Idling on the ground is not hover, even after in_air latched.
  state.altitude = 0.0f;
  for (int i = 0; i < 300; i++) nav_test_update_hover(0.01f);
  TEST_ASSERT_EQUAL_FLOAT(0, state.hover_throttle);
  state.altitude = 5.0f;
  for (int i = 0; i < 100; i++) nav_test_update_hover(0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, state.hover_throttle);
  // Climbing, tilted, or automatic flight is not steady hover.
  state.throttle = 0.7f;
  state.vertical_speed = 2;
  for (int i = 0; i < 300; i++) nav_test_update_hover(0.01f);
  state.vertical_speed = 0;
  state.attitude.pitch = 30 * DEGTORAD;
  for (int i = 0; i < 300; i++) nav_test_update_hover(0.01f);
  state.attitude.pitch = 0;
  flags.controls_override = 1;
  for (int i = 0; i < 300; i++) nav_test_update_hover(0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.4f, state.hover_throttle);
  // Battery sag: the estimate follows the new steady throttle.
  flags.controls_override = 0;
  state.throttle = 0.5f;
  for (int i = 0; i < 2000; i++) nav_test_update_hover(0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, state.hover_throttle);
  // A new flight starts without the previous battery's estimate.
  flags.arm_state = 0;
  nav_test_update_hover(0.01f);
  TEST_ASSERT_EQUAL_FLOAT(0, state.hover_throttle);
}

void test_navigation_learned_hover_trims_rth(void) {
  start_test_rth();
  nav_rth_stop();
  nav_test_set_hover_learned(0.3f);
  state.throttle = 0.3f;
  nav_rth_start();
  for (int i = 0; i < 500; i++) nav_test_altitude_control(0, 2, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.3f, state.rx_override.throttle);
}

void test_navigation_vertical_speed_fusion(void) {
  navigation_test_setup();
  time_test_set_us(1000000);
  // Accelerate upward at 2 m/s^2 for one second with 20 Hz barometer samples.
  float height = 0, velocity = 0;
  for (int i = 0; i < 100; i++) {
    velocity += 2 * 0.01f;
    height += velocity * 0.01f;
    time_test_advance_us(10000);
    if (i % 5 == 0) nav_test_altitude_sample(height, time_millis());
    sim_vertical(2, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.15f, velocity, state.vertical_speed);
  TEST_ASSERT_TRUE(fabsf(state.baro_vertical_speed - velocity) > fabsf(state.vertical_speed - velocity));

  // A stationary craft with accelerometer bias converges to zero speed.
  navigation_test_setup();
  time_test_set_us(1000000);
  for (int i = 0; i < 3000; i++) {
    time_test_advance_us(10000);
    if (i % 5 == 0) nav_test_altitude_sample(0, time_millis());
    sim_vertical(0.3f, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0, state.vertical_speed);

  // Without barometer the integral leaks instead of running away: a residual
  // bias settles at bias * 5s rather than growing without bound.
  set_altitude_source(false, 0);
  for (int i = 0; i < 6000; i++) {
    time_test_advance_us(10000);
    sim_vertical(0.35f, 0.01f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.25f, state.vertical_speed);
}

void test_navigation_closed_loop_return_with_irregular_samples(void) {
  start_test_rth();
  // Simple point-mass plant with attitude lag and steady lateral disturbance.
  // This exercises navigation and altitude control, not the inner PID or AHRS.
  float north = 100, east = 20, height = 0;
  float vn = 0, ve = 0, vz = 0, pitch = 0, roll = 0, heading = 0;
  uint32_t next_gps_ms = time_millis();
  uint32_t next_baro_ms = time_millis();
  unsigned gps_samples = 0, baro_samples = 0;
  float max_height = 0;
  for (int i = 0; i < 12000; i++) {
    const float dt = 0.01f;
    time_test_advance_us(10000);
    if (time_millis() >= next_gps_ms) {
      state.gps_coord.lat = lrintf(north * 89.93f);
      state.gps_coord.lon = lrintf(east * 89.93f);
      state.gps_vel_north = vn;
      state.gps_vel_east = ve;
      state.gps_speed = hypotf(vn, ve);
      state.home_distance = hypotf(north, east);
      state.home_bearing = normalize_deg(atan2f(-east, -north) * RADTODEG);
      state.gps_last_update_ms = time_millis();
      next_gps_ms += gps_samples++ % 2 ? 80 : 120;
    }
    if (time_millis() >= next_baro_ms) {
      nav_test_altitude_sample(height, time_millis());
      next_baro_ms += baro_samples++ % 2 ? 40 : 60;
    }
    state.heading = normalize_deg(heading * RADTODEG);
    state.attitude.pitch = pitch;
    state.attitude.roll = roll;
    const auto phase = state.rth_state;
    nav_test_update_rth();
    char context[120];
    snprintf(context, sizeof(context), "Return aborted: t=%.2f phase=%u distance=%.2f altitude=%.2f", (double)(i * dt), (unsigned)phase, (double)state.home_distance, (double)height);
    TEST_ASSERT_TRUE_MESSAGE(state.rth_active, context);
    pitch += dt / 0.1f * (state.rx_override.pitch * 60 * DEGTORAD - pitch);
    roll += dt / 0.1f * (state.rx_override.roll * 60 * DEGTORAD - roll);
    heading += state.rth_yaw_rate * dt;
    const float af = 9.81f * tanf(pitch);
    const float ar = 9.81f * tanf(roll);
    vn += dt * (af * cosf(heading) - ar * sinf(heading) - 0.12f * vn);
    ve += dt * (af * sinf(heading) + ar * cosf(heading) - 0.12f * ve + 0.15f);
    const float az = 9.81f * (state.rx_override.throttle / 0.5f * cosf(pitch) * cosf(roll) - 1) - 0.3f * vz;
    vz += dt * az;
    sim_vertical(az, dt);
    north += vn * dt;
    east += ve * dt;
    height += vz * dt;
    max_height = MAX(max_height, height);
  }
  TEST_ASSERT_EQUAL(RTH_STATE_HOVER_HOME, state.rth_state);
  TEST_ASSERT_TRUE(hypotf(north, east) < 3);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 10, height);
  TEST_ASSERT_TRUE(max_height < 12);
}

void test_navigation_turns_before_horizontal_commands() {
  start_test_rth();
  state.home_bearing = 180;
  state.altitude = 10;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
  for (int i = 0; i < 100; i++) {
    rth_step(10000);
    TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.roll);
    TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 30 * DEGTORAD, state.rth_yaw_rate);
  state.heading = 175;
  state.gyro.yaw = 15 * DEGTORAD;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
  state.gyro.yaw = 0;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAVIGATE, state.rth_state);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  rth_step(10000);
  TEST_ASSERT_TRUE(state.rx_override.pitch > 0);
}

void test_navigation_turn_wraps_heading_and_respects_rate() {
  start_test_rth();
  state.altitude = 10;
  state.heading = 10;
  state.home_bearing = 350;
  rth_step(10000);
  for (int i = 0; i < 100; i++) rth_step(10000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -20 * DEGTORAD, state.rth_yaw_rate);
  state.heading = 350;
  state.home_bearing = 10;
  for (int i = 0; i < 100; i++) rth_step(10000);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20 * DEGTORAD, state.rth_yaw_rate);
}

void test_navigation_stalled_turn_aborts() {
  start_test_rth();
  state.altitude = 10;
  state.home_bearing = 180;
  rth_step(10000);
  for (int i = 0; i < 1601; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAV_FAILED, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rth_yaw_rate);
  for (int i = 0; i < 1000; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_ABORTED, state.rth_state);
}

void test_navigation_heading_loss_stops_horizontal_commands() {
  start_test_rth();
  state.altitude = 10;
  rth_step(10000);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_NAVIGATE, state.rth_state);
  state.rx_override.roll = state.rx_override.pitch = 0.2f;
  state.heading = 90;
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.roll);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
}

void test_navigation_acquires_heading_before_turning() {
  const struct { float heading; int sample_loops; float max_angle; } cases[] = {
      {0, 10, 35}, {90, 10, 35}, {180, 10, 35}, {180, 20, 35}, {180, 10, 15}, {180, 10, 45}};
  for (const auto &scenario : cases) {
    const float true_heading = scenario.heading;
    start_test_rth();
    profile.rate.level_max_angle = scenario.max_angle;
    state.heading_confidence = 0;
    state.altitude = 10;
    state.home_bearing = normalize_deg(true_heading + 90);
    rth_step(10000);
    TEST_ASSERT_EQUAL(RTH_STATE_ACQUIRE_HEADING, state.rth_state);
    float pitch = 0, speed = 0, north = 0, east = 0;
    state.looptime = 0.01f;
    for (int i = 0; i < 1400 && state.rth_state == RTH_STATE_ACQUIRE_HEADING; i++) {
      const float previous_pitch = pitch;
      pitch += 0.01f / 0.2f * (state.rx_override.pitch * profile.rate.level_max_angle * DEGTORAD - pitch);
      state.gyro.pitch = (pitch - previous_pitch) / 0.01f;
      state.GEstG.pitch = sinf(pitch);
      state.GEstG.yaw = cosf(pitch);
      speed += 0.01f * (9.81f * tanf(pitch) - 0.3f * speed);
      north += speed * cosf(true_heading * DEGTORAD) * 0.01f;
      east += speed * sinf(true_heading * DEGTORAD) * 0.01f;
      if (i % scenario.sample_loops == 0) {
        state.gps_last_update_ms = time_millis();
        state.gps_speed = speed;
        state.gps_heading = true_heading;
        state.gps_heading_accuracy = 1;
        state.gps_coord.lat = 8990 + int32_t(north * 89.9f);
        state.gps_coord.lon = int32_t(east * 89.9f);
      }
      imu_test_attitude_update();
      rth_step(10000);
      TEST_ASSERT_TRUE(state.rth_active);
      TEST_ASSERT_EQUAL_FLOAT(0, state.rth_yaw_rate);
      TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.roll);
      TEST_ASSERT_TRUE(state.rx_override.pitch <= MIN(35.0f, profile.rate.level_max_angle) / profile.rate.level_max_angle + 0.001f);
    }
    TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
    TEST_ASSERT_TRUE(state.heading_confidence >= 0.6f);
    TEST_ASSERT_FLOAT_WITHIN(5, true_heading, state.heading);
    TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
    rth_step(10000);
    TEST_ASSERT_EQUAL(RTH_STATE_TURN, state.rth_state);
    TEST_ASSERT_TRUE(state.rth_yaw_rate > 0);
    TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  }
}

void test_navigation_acquisition_failure_holds_and_can_cancel() {
  start_test_rth();
  state.heading_confidence = 0;
  state.altitude = 10;
  state.rth_failsafe_active = true;
  rth_step(10000);
  for (int i = 0; i < 1501; i++) rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_HEADING_FAILED, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_TRUE(state.rth_failsafe_active);
  TEST_ASSERT_TRUE(flags.controls_override);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rth_yaw_rate);
  TEST_ASSERT_TRUE(state.rx_override.throttle >= profile.navigation.rth_throttle_min);
  nav_rth_stop();
  TEST_ASSERT_FALSE(flags.controls_override);
  TEST_ASSERT_EQUAL(RTH_STATE_INACTIVE, state.rth_state);
}

void test_navigation_acquisition_distance_limit() {
  start_test_rth();
  state.heading_confidence = 0;
  state.altitude = 10;
  rth_step(10000);
  state.gps_coord.lon += 7200; // About 80m at the equator.
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_HEADING_FAILED, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
}

void test_navigation_gps_outage_does_not_extend_acquisition() {
  start_test_rth();
  state.heading_confidence = 0;
  state.altitude = 10;
  rth_step(10000);
  for (int i = 0; i < 1400; i++) rth_step(10000);
  nav_test_set_gps_sane(false);
  rth_step(10000);
  TEST_ASSERT_EQUAL_FLOAT(0, state.rx_override.pitch);
  rth_step(2000000);
  nav_test_set_gps_sane(true);
  rth_step(10000);
  TEST_ASSERT_EQUAL(RTH_STATE_HEADING_FAILED, state.rth_state);
  TEST_ASSERT_TRUE(state.rth_active);
}

// Exercise the horizontal controller with receiver delay, sample jitter,
// velocity noise, airframe drag and finite attitude response.
void test_navigation_delayed_gps_cruise_and_arrival() {
  const float speeds[] = {4.0f, 30.0f / 3.6f};
  const int delays[] = {20, 35};
  for (float cruise : speeds) {
    for (int delay : delays) {
      navigation_test_setup();
      profile.navigation.rth_cruise_speed = cruise;
      profile.rate.level_max_angle = 65;
      state.heading_confidence = 0.5f;
      state.rth_state = RTH_STATE_NAVIGATE;
      float position = -400, velocity = 0, pitch = 0;
      float positions[64], velocities[64];
      for (int i = 0; i < 64; i++) { positions[i] = position; velocities[i] = 0; }
      float max_overshoot = 0, arrival_speed = -1, cruise_sum = 0, pitch_rate_square = 0;
      int cruise_samples = 0, next_sample = 0, sample_count = 0;
      for (int i = 0; i < 18000; i++) {
        const float dt = 0.01f;
        const float t = i * dt;
        positions[i % 64] = position;
        velocities[i % 64] = velocity;
        time_test_advance_us(10000);
        if (i >= next_sample) {
          const int index = (i + 64 - delay) % 64;
          state.gps_coord.lat = lrintf(positions[index] * 89.93f);
          state.gps_vel_north = velocities[index] + 0.3f * sinf(8 * t) + 0.15f * sinf(19 * t);
          state.gps_last_update_ms = time_millis();
          next_sample += sample_count++ % 2 ? 8 : 12;
        }
        nav_test_update_horizontal_control(dt);
        const float previous_pitch = pitch;
        pitch += dt / 0.2f * (state.rx_override.pitch * 65 * DEGTORAD - pitch);
        velocity += dt * (9.81f * tanf(pitch) - 0.45f * velocity);
        position += velocity * dt;
        if (fabsf(position) < 3 && arrival_speed < 0) arrival_speed = fabsf(velocity);
        max_overshoot = MAX(max_overshoot, position);
        if (t > 25 && t < 35) {
          cruise_sum += velocity;
          const float pitch_rate = (pitch - previous_pitch) * RADTODEG / dt;
          pitch_rate_square += pitch_rate * pitch_rate;
          cruise_samples++;
        }
      }
      printf("cruise=%.2f delay=%d speed=%.2f pitch_rate_rms=%.2f arrival=%.2f overshoot=%.2f final=%.2f\n",
          (double)cruise, delay, (double)(cruise_sum / cruise_samples),
          (double)sqrtf(pitch_rate_square / cruise_samples), (double)arrival_speed,
          (double)max_overshoot, (double)position);
      TEST_ASSERT_FLOAT_WITHIN(0.6f, cruise, cruise_sum / cruise_samples);
      TEST_ASSERT_TRUE(sqrtf(pitch_rate_square / cruise_samples) < 8);
      TEST_ASSERT_TRUE(arrival_speed >= 0 && arrival_speed < 2);
      TEST_ASSERT_TRUE(max_overshoot < 3);
      TEST_ASSERT_FLOAT_WITHIN(1, 0, position);
    }
  }
}

extern void test_navigation_motor_loop_nominal();
extern void test_navigation_plant_specific_force_and_motor_signs();
extern void test_navigation_motor_loop_delayed_gps_and_wind();
extern void test_navigation_motor_loop_wrong_hover();
extern void test_navigation_motor_loop_sensor_outages();
extern void test_navigation_motor_loop_rc_loss_and_handback();

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_navigation_target_north_commands_forward_pitch);
  RUN_TEST(test_navigation_target_south_commands_backward_pitch);
  RUN_TEST(test_navigation_target_east_commands_right_roll);
  RUN_TEST(test_navigation_heading_east_target_north_commands_left_roll);
  RUN_TEST(test_navigation_north_velocity_overshoot_commands_backward_pitch);
  RUN_TEST(test_navigation_east_velocity_overshoot_commands_left_roll);
  RUN_TEST(test_navigation_command_respects_level_angle_limit);
  RUN_TEST(test_navigation_rth_requires_recent_altitude_source);
  RUN_TEST(test_navigation_resets_gps_jump_baseline_after_outage);
  RUN_TEST(test_navigation_yaw_rate_bypasses_pilot_curves);
  RUN_TEST(test_navigation_climb_publishes_gps_acceleration);
  RUN_TEST(test_navigation_throttle_bypasses_pilot_curves);
  RUN_TEST(test_navigation_vertical_rate_uses_sample_interval);
  RUN_TEST(test_navigation_altitude_integral_unwinds_after_saturation);
  RUN_TEST(test_navigation_descent_uses_configured_throttle_minimum);
  RUN_TEST(test_navigation_rth_accepts_unestablished_heading);
  RUN_TEST(test_navigation_acquires_heading_before_turning);
  RUN_TEST(test_navigation_acquisition_failure_holds_and_can_cancel);
  RUN_TEST(test_navigation_acquisition_distance_limit);
  RUN_TEST(test_navigation_gps_outage_does_not_extend_acquisition);
  RUN_TEST(test_navigation_altitude_recovers_with_wrong_hover_setting);
  RUN_TEST(test_navigation_altitude_handles_delay_and_noise);
  RUN_TEST(test_navigation_altitude_takeover_is_smooth);
  RUN_TEST(test_navigation_pitch_command_matches_imu_and_angle_control);
  RUN_TEST(test_navigation_integral_stays_earth_referenced_during_yaw);
  RUN_TEST(test_navigation_repeated_gps_sample_keeps_commands_smooth);
  RUN_TEST(test_navigation_altitude_loss_holds_and_resumes);
  RUN_TEST(test_navigation_nonfinite_altitude_holds);
  RUN_TEST(test_navigation_altitude_loss_holds_vertical_speed);
  RUN_TEST(test_navigation_gps_loss_holds_latched_altitude);
  RUN_TEST(test_navigation_flapping_gps_does_not_extend_return);
  RUN_TEST(test_navigation_gps_recovery_resumes_original_climb);
  RUN_TEST(test_navigation_stalled_climb_and_return_abort);
  RUN_TEST(test_navigation_hover_does_not_timeout);
  RUN_TEST(test_navigation_failsafe_rth_waits_for_stable_link_and_sticks);
  RUN_TEST(test_navigation_flapping_link_does_not_interrupt_rth);
  RUN_TEST(test_navigation_failsafe_rth_gives_up_with_stage2);
  RUN_TEST(test_navigation_failsafe_rth_disarms_after_crash);
  RUN_TEST(test_navigation_failsafe_holds_pilot_throttle_instead_of_idle);
  RUN_TEST(test_navigation_failsafe_on_ground_keeps_idle);
  RUN_TEST(test_navigation_failsafe_hold_levels_in_acro);
  RUN_TEST(test_navigation_failsafe_rth_takeover_does_not_drop);
  RUN_TEST(test_navigation_wrong_learned_hover_does_not_drop);
  RUN_TEST(test_navigation_failsafe_on_ground_does_not_start_rth);
  RUN_TEST(test_navigation_rth_starts_close_to_home);
  RUN_TEST(test_navigation_hover_learning);
  RUN_TEST(test_navigation_learned_hover_trims_rth);
  RUN_TEST(test_navigation_vertical_speed_fusion);
  RUN_TEST(test_navigation_closed_loop_return_with_irregular_samples);
  RUN_TEST(test_navigation_delayed_gps_cruise_and_arrival);
  RUN_TEST(test_navigation_turns_before_horizontal_commands);
  RUN_TEST(test_navigation_turn_wraps_heading_and_respects_rate);
  RUN_TEST(test_navigation_stalled_turn_aborts);
  RUN_TEST(test_navigation_heading_loss_stops_horizontal_commands);
  RUN_TEST(test_navigation_plant_specific_force_and_motor_signs);
  RUN_TEST(test_navigation_motor_loop_nominal);
  RUN_TEST(test_navigation_motor_loop_delayed_gps_and_wind);
  RUN_TEST(test_navigation_motor_loop_wrong_hover);
  RUN_TEST(test_navigation_motor_loop_sensor_outages);
  RUN_TEST(test_navigation_motor_loop_rc_loss_and_handback);
  return UNITY_END();
}
