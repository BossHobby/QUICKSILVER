#include <unity.h>
#include <math.h>

#include "control/control.h"
#include "control/rover/control.h"
#include "core/profile.h"
#include "driver/time.h"
#include "mock_outputs.h"

extern const profile_t default_profile;

void setUp() {
  profile = default_profile;
  profile.motor.motor_limit = 100;
  state = {};
  flags = {};
  target = {};
  time_test_set_us(1000000);
  state.looptime = 0.001f;
  state.looptime_inverse = 1000.0f;
  state.looptime_autodetect = 1000.0f;
  state.GEstG.yaw = 1.0f;
  state.accel_raw.yaw = 1.0f;
  pid_init();
  state.rx_filtered.throttle = 0.5f;
  flags.rx_ready = 1;
  control_update_arming();
  control_update_arming();
  target.outputs[0].pin = PIN_A0;
  target.outputs[0].caps = OUTPUT_CAP_PWM;
  target.outputs[3].pin = PIN_A3;
  target.outputs[3].caps = OUTPUT_CAP_PWM;
  servo_init();
}
void tearDown() {}

static void test_reversible_drive_and_steering() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[0]);
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 1.0f;
  state.rx_filtered.yaw = 0.3f;
  control();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pwm_values[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, pwm_values[3]);
  state.rx_filtered.throttle = 0.0f;
  control();
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  state.aux_active = 0;
  control();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[0]);
}

static void check_rate_throttle_cut(float rx_throttle, float direction) {
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | (1U << AUX_RATE_THROTTLE);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);

  // Request 10% drive while yaw motion calls for a larger throttle cut.
  state.rx_filtered.throttle = rx_throttle;
  state.gyro.yaw = direction * 2.0f;
  for (unsigned i = 0; i < 1000; i++) {
    control();
    TEST_ASSERT_TRUE(flags.arm_state);
    TEST_ASSERT_TRUE(direction * pwm_values[0] >= 0.0f);
  }
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[0]);

  // Removing the yaw disturbance restores drive in the requested direction.
  state.gyro.yaw = 0.0f;
  for (unsigned i = 0; i < 1000; i++) {
    control();
  }
  TEST_ASSERT_FLOAT_WITHIN(0.001f, direction * 0.1f, pwm_values[0]);
}

static void test_rate_throttle_cut_preserves_forward_direction() {
  check_rate_throttle_cut(0.595f, 1.0f);
}

static void test_rate_throttle_cut_preserves_reverse_direction() {
  check_rate_throttle_cut(0.405f, -1.0f);
}

static void arm_rate_assist() {
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | (1U << AUX_RATE_ASSIST);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);
}

static void run_control(unsigned count) {
  for (unsigned i = 0; i < count; i++) {
    control();
  }
}

static void enable_motion_scaling() {
  profile.rover.throttle_scale_breakpoint = 0.0f;
  profile.rover.throttle_scale_factor = 0.0f;
  state.rx_filtered.yaw = 1.0f;
}

static float yaw_sensitivity() {
  return state.setpoint.yaw / (profile.rover.yaw_rate * (3.14159265f / 180.0f));
}

static void test_motion_rejects_gravity_on_slope() {
  arm_rate_assist();
  enable_motion_scaling();
  state.rx_filtered.throttle = 1.0f;
  run_control(500);
  const float flat_sensitivity = yaw_sensitivity();

  pid_init();
  state.GEstG.pitch = 0.5f;
  state.GEstG.yaw = sqrtf(0.75f);
  state.accel_raw = state.GEstG;
  // The separately normalized IMU accel must not influence this estimator.
  state.accel.pitch = -1.0f;
  run_control(500);
  TEST_ASSERT_FLOAT_WITHIN(0.00001f, flat_sensitivity, yaw_sensitivity());
}

static void test_acceleration_speeds_up_motion_response() {
  arm_rate_assist();
  enable_motion_scaling();
  state.rx_filtered.throttle = 1.0f;
  run_control(500);
  const float without_acceleration = yaw_sensitivity();
  pid_init();
  state.accel_raw.pitch = -0.5f;
  run_control(500);
  TEST_ASSERT_TRUE(yaw_sensitivity() < without_acceleration - 0.2f);
}

static float sensitivity_after_lift(float direction, float accel_forward) {
  pid_init();
  state.rx_filtered.throttle = direction > 0 ? 1.0f : 0.0f;
  state.accel_raw.pitch = 0.0f;
  run_control(10000);
  TEST_ASSERT_TRUE(yaw_sensitivity() < 0.02f);
  state.rx_filtered.throttle = 0.5f;
  state.accel_raw.pitch = -accel_forward;
  run_control(500);
  return yaw_sensitivity();
}

static void test_coasting_retains_scaling_and_braking_releases_it() {
  arm_rate_assist();
  enable_motion_scaling();
  const float coast = sensitivity_after_lift(1.0f, 0.0f);
  const float brake = sensitivity_after_lift(1.0f, -0.5f);
  TEST_ASSERT_TRUE(coast < 0.3f);
  TEST_ASSERT_TRUE(brake > coast + 0.3f);
  const float reverse_brake = sensitivity_after_lift(-1.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, brake, reverse_brake);
}

static void test_motion_is_bounded_and_resets_on_disarm() {
  arm_rate_assist();
  enable_motion_scaling();
  state.rx_filtered.throttle = 1.0f;
  state.accel_raw.pitch = -100.0f;
  run_control(10000);
  TEST_ASSERT_TRUE(yaw_sensitivity() >= 0.0f && yaw_sensitivity() <= 1.0f);
  state.aux_active = 0;
  control();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, yaw_sensitivity());
}

static void test_scaled_yaw_demand_preserves_countersteering() {
  arm_rate_assist();
  profile.rover.throttle_scale_breakpoint = 0.0f;
  profile.rover.throttle_scale_factor = 0.25f;
  profile.rover.pid.kd = 0.0f;
  profile.rover.pid.ki = 0.0f;
  state.rx_filtered.throttle = 1.0f;
  state.rx_filtered.yaw = 1.0f;
  run_control(20000);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, profile.rover.yaw_rate * (3.14159265f / 180.0f) * 0.25f, state.setpoint.yaw);

  state.rx_filtered.yaw = 0.0f;
  state.gyro.yaw = 20.0f;
  control();
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[3]);
  state.gyro.yaw = -20.0f;
  control();
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pwm_values[3]);

  // Manual steering retains travel scaling, independent of measured yaw.
  state.aux_active &= ~(1U << AUX_RATE_ASSIST);
  state.rx_filtered.yaw = 1.0f;
  control();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, pwm_values[3]);
}

static void test_default_scaling_leaves_yaw_demand_unchanged() {
  arm_rate_assist();
  state.rx_filtered.throttle = 1.0f;
  state.rx_filtered.yaw = 0.5f;
  run_control(10000);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, profile.rover.yaw_rate * (3.14159265f / 180.0f) * 0.5f, state.setpoint.yaw);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reversible_drive_and_steering);
  RUN_TEST(test_rate_throttle_cut_preserves_forward_direction);
  RUN_TEST(test_rate_throttle_cut_preserves_reverse_direction);
  RUN_TEST(test_motion_rejects_gravity_on_slope);
  RUN_TEST(test_acceleration_speeds_up_motion_response);
  RUN_TEST(test_coasting_retains_scaling_and_braking_releases_it);
  RUN_TEST(test_motion_is_bounded_and_resets_on_disarm);
  RUN_TEST(test_scaled_yaw_demand_preserves_countersteering);
  RUN_TEST(test_default_scaling_leaves_yaw_demand_unchanged);
  return UNITY_END();
}
