#include <unity.h>

#include "control/control.h"
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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_reversible_drive_and_steering);
  return UNITY_END();
}
