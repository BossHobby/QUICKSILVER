#include <math.h>
#include <string.h>
#include <unity.h>

#include "control/control.h"
#include "control/multi/navigation.h"
#include "core/profile.h"
#include "mock_helpers.h"

extern void nav_test_reset(void);
extern void nav_test_update_horizontal_control(float dt);

#define GPS_DEG_10M 899

static void navigation_test_setup(void) {
  mock_hardware_reset_all();
  memset(&state, 0, sizeof(state));
  memset(&flags, 0, sizeof(flags));

  state.gps_lock = true;
  state.gps_sats = GPS_MIN_SATS_FOR_LOCK;
  state.heading = 0.0f;
  state.gps_heading = 0.0f;
  state.gps_speed = 0.0f;
  profile.rate.level_max_angle = 60.0f;
  profile.navigation.rth_cruise_speed = 8.0f;
  profile.navigation.rth_home_radius = 3.0f;
  profile.navigation.rth_max_angle = 25.0f;
  profile.navigation.rth_max_yaw_rate = 45.0f;
  profile.navigation.rth_angle_slew_rate = 100.0f;
  profile.navigation.nav_vel_kp = 0.12f;
  profile.navigation.nav_vel_ki = 0.02f;
  profile.navigation.nav_vel_kd = 0.03f;
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

void test_navigation_command_respects_rth_angle_limit(void) {
  navigation_test_setup();

  profile.rate.level_max_angle = 60.0f;
  profile.navigation.rth_max_angle = 30.0f;
  profile.navigation.nav_vel_kp = 10.0f;
  profile.navigation.nav_vel_ki = 0.0f;
  profile.navigation.nav_vel_kd = 0.0f;

  state.gps_coord.lat = 0;
  state.gps_coord.lon = 0;
  state.gps_home.lat = GPS_DEG_10M;
  state.gps_home.lon = 0;

  for (int i = 0; i < 30; i++) {
    nav_test_update_horizontal_control(0.1f);
  }

  TEST_ASSERT_TRUE(state.rx_override.pitch > 0.0f);
  TEST_ASSERT_TRUE(state.rx_override.pitch <= 15.0f / 60.0f + 0.01f);
}
