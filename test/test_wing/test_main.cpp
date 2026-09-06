#include "mock_outputs.h"
#include <unity.h>

#include "control/control.h"
#include "control/output.h"
#include "control/pid.h"
#include "core/failloop.h"
#include "core/profile.h"
#include "driver/motor.h"
#include "driver/servo.h"
#include "driver/time.h"
#include "osd/render.h"

extern const profile_t default_profile;
void setUp() {
  profile = default_profile;
  state = {};
  flags = {};
  target = {};
  osd_state.screen_history_size = 0;
  fault = FAILLOOP_NONE;
  pwm_rate = 0;
  time_test_set_us(1000000);
  flags.rx_ready = 1;
  profile.receiver.aux[AUX_PREARM].channel = RX_CHANNEL_ON;
  control_update_arming();
  control_update_arming();
  state.looptime = 0.001f;
  state.looptime_inverse = 1000.0f;
  state.GEstG.yaw = 1.0f;
  pid_init();
  for (unsigned i = 0; i < 4; i++) {
    target.outputs[i].pin = (gpio_pins_t)(PIN_A0 + i);
    target.outputs[i].caps = OUTPUT_CAP_PWM | OUTPUT_CAP_DSHOT;
  }
}
void tearDown() {}

static void pwm_motor() {
  profile.outputs[0].protocol = OUTPUT_PROTOCOL_PWM;
  profile.outputs[0].rate_hz = 50;
  servo_init();
}
static void write_throttle(float value) {
  state.mixer_source[OUTPUT_SOURCE_THROTTLE] = value;
  output_apply_mixer_rules();
  output_finalize_motor_values();
  output_write_values();
  output_write_all();
}
static void test_pwm_throttle_range_and_limit() {
  pwm_motor();
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  flags.arm_state = 1;
  profile.motor.motor_limit = 100;
  write_throttle(0.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  write_throttle(0.5f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[0]);
  write_throttle(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pwm_values[0]);
  profile.motor.motor_limit = 50;
  write_throttle(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, state.thrsum);
}
static void test_pwm_motor_stops_but_disarmed_surfaces_work() {
  pwm_motor();
  state.mixer_source[OUTPUT_SOURCE_ROLL] = 0.3f;
  write_throttle(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, pwm_values[1]);
  flags.arm_state = 1;
  flags.failsafe_outputs_blocked = 1;
  write_throttle(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pwm_values[1]);
  flags.failsafe_outputs_blocked = 0;
  write_throttle(1.0f);
  output_stop_all();
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
}
static void test_pwm_shared_rate_validation() {
  profile.outputs[1].rate_hz = 333;
  servo_init();
  TEST_ASSERT_EQUAL(FAILLOOP_OUTPUT, fault);
  TEST_ASSERT_EQUAL(0, pwm_rate);
  for (unsigned i = 1; i < 4; i++)
    profile.outputs[i].rate_hz = 333;
  fault = FAILLOOP_NONE;
  servo_init();
  TEST_ASSERT_EQUAL(FAILLOOP_NONE, fault);
  TEST_ASSERT_EQUAL(333, pwm_rate);
  profile.outputs[1].rate_hz = 0;
  servo_init();
  TEST_ASSERT_EQUAL(FAILLOOP_OUTPUT, fault);
}
static void arm() {
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);
}
static void test_gliding_retains_airborne_state_and_motor_cutoff() {
  arm();
  state.rx_filtered.throttle = 0.5f;
  control();
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_FALSE(flags.on_ground);
  state.rx_filtered.throttle = 0.0f;
  control();
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_FALSE(flags.on_ground);
  TEST_ASSERT_EQUAL_FLOAT(MOTOR_OFF, state.output[0]);
  state.aux_active = 0;
  control();
  TEST_ASSERT_FALSE(flags.in_air);
  TEST_ASSERT_TRUE(flags.on_ground);
}
static void test_autotrim_resets_only_trimmed_axes_with_servo_inversion() {
  arm();
  state.aux_active |= (1U << AUX_ACROMODE) | (1U << AUX_AUTOTRIM);
  flags.in_air = 1;
  profile.outputs[1].invert = true;
  profile.wing.autotrim.step = 0.01f;
  profile.wing.autotrim.threshold = 0.04f;
  // Accumulate simultaneous corrections through the PID before running autotrim.
  profile.pid.pid_rates[0].ki = (vec3_t){{100.0f, 100.0f, 100.0f}};
  state.error = (vec3_t){{0.1f, 0.2f, -0.15f}};
  for (unsigned i = 0; i < 1000; i++)
    pid_calc();
  profile.pid.pid_rates[0].ki = {};

  control();
  TEST_ASSERT_EQUAL_INT(-20, profile.outputs[1].trim);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[2].trim);
  TEST_ASSERT_EQUAL_INT(-10, profile.outputs[3].trim);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->yaw);

  pid_reset_i();
  profile.pid.pid_rates[0].ki = (vec3_t){{100.0f, 100.0f, 100.0f}};
  state.error = (vec3_t){{0.1f, 0.01f, -0.01f}};
  for (unsigned i = 0; i < 1000; i++)
    pid_calc();
  profile.pid.pid_rates[0].ki = {};
  const vec3_t before = *pid_get_ierror();
  time_test_advance_us(500000);
  control();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, before.pitch, pid_get_ierror()->pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, before.yaw, pid_get_ierror()->yaw);
}
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_pwm_throttle_range_and_limit);
  RUN_TEST(test_pwm_motor_stops_but_disarmed_surfaces_work);
  RUN_TEST(test_pwm_shared_rate_validation);
  RUN_TEST(test_gliding_retains_airborne_state_and_motor_cutoff);
  RUN_TEST(test_autotrim_resets_only_trimmed_axes_with_servo_inversion);
  return UNITY_END();
}
