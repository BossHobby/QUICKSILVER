#include <unity.h>

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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reversible_drive_and_steering);
  RUN_TEST(test_rate_throttle_cut_preserves_forward_direction);
  RUN_TEST(test_rate_throttle_cut_preserves_reverse_direction);
  return UNITY_END();
}
