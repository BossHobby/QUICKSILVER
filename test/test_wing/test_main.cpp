#include "mock_outputs.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unity.h>

#include "control/control.h"
#include "control/output.h"
#include "control/pid.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/motor.h"
#include "driver/servo.h"
#include "driver/time.h"
#include "osd/render.h"

extern const profile_t default_profile;
extern void run_wing_safety_tests();
void setUp() {
  profile = default_profile;
  profile_output_update();
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
  profile_output_update();
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
  servo_init();
  TEST_ASSERT_EQUAL(50, pwm_rate);
  profile.servo.pwm_rate_hz = 334;
  pwm_rate = 0;
  servo_init();
  TEST_ASSERT_EQUAL(FAILLOOP_OUTPUT, fault);
  TEST_ASSERT_EQUAL(0, pwm_rate);
  profile.servo.pwm_rate_hz = 333;
  fault = FAILLOOP_NONE;
  servo_init();
  TEST_ASSERT_EQUAL(FAILLOOP_NONE, fault);
  TEST_ASSERT_EQUAL(333, pwm_rate);
  profile.servo.pwm_rate_hz = 0;
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
static void start_autotrim() {
  state.aux_active |= (1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
}

static void finish_autotrim() {
  time_test_advance_us(2000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVE_PENDING, state.wing_autotrim_state);
}

static void test_autotrim_requires_switch_and_averages_applied_surfaces_once() {
  arm();
  pwm_motor();
  profile.outputs[1].invert = true;
  profile.outputs[1].trim = 100;
  profile.outputs[1].min = -400;
  profile.outputs[3].trim = 20;
  state.rx_filtered.roll = 0.2f;
  state.rx_filtered.pitch = 0.1f;
  state.rx_filtered.yaw = -0.1f;
  control();
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(100, profile.outputs[1].trim);
  start_autotrim(); // Applied elevons: -0.2, -0.1; rudder: -0.08.
  state.rx_filtered.roll = 0.4f;
  state.rx_filtered.pitch = 0.2f;
  time_test_advance_us(1999000);
  control(); // Applied elevons: -0.4 (limited), -0.2.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(100, profile.outputs[1].trim);
  time_test_advance_us(1000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVE_PENDING, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(-333, profile.outputs[1].trim);
  TEST_ASSERT_EQUAL_INT(-167, profile.outputs[2].trim);
  TEST_ASSERT_EQUAL_INT(-80, profile.outputs[3].trim);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[0].trim);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.4f, pwm_values[1]);
  state.rx_filtered = {};
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL_INT(-333, profile.outputs[1].trim);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.333f, pwm_values[1]);
  state.aux_active &= ~(1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_EQUAL_INT(100, profile.outputs[1].trim);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[2].trim);
  TEST_ASSERT_EQUAL_INT(20, profile.outputs[3].trim);
  start_autotrim();
}

static void test_autotrim_captures_integral_correction_before_reset() {
  arm();
  state.aux_active |= (1U << AUX_ACROMODE);
  flags.in_air = 1;
  profile.pid.pid_rates[0].ki = (vec3_t){{100.0f, 100.0f, 100.0f}};
  pid_rates_update();
  state.error = (vec3_t){{0.05f, 0.1f, -0.075f}};
  for (unsigned i = 0; i < 1000; i++)
    pid_calc();
  profile.pid.pid_rates[0].ki = {};
  pid_rates_update();
  start_autotrim();
  const float before = pwm_values[1];
  TEST_ASSERT_TRUE(before > 0.1f);
  finish_autotrim();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, before, pwm_values[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->roll);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, pid_get_ierror()->yaw);

  control();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, before, pwm_values[1]);
}

static void test_autotrim_cancel_during_capture_and_on_early_disarm() {
  arm();
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  time_test_advance_us(1000000);
  state.aux_active &= ~(1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[1].trim);
  start_autotrim();
  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[1].trim);
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
}

static void test_autotrim_saves_on_disarm_with_switch_on() {
  arm();
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  finish_autotrim();
  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVED, state.wing_autotrim_state);
  profile_t saved = {};
  fmc_read_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  TEST_ASSERT_EQUAL_INT(200, saved.outputs[1].trim);
  TEST_ASSERT_EQUAL_INT(-200, saved.outputs[2].trim);
  // No repeated save while the switch remains on.
  profile.outputs[1].trim = 123;
  control();
  fmc_read_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  TEST_ASSERT_EQUAL_INT(200, saved.outputs[1].trim);
  state.aux_active &= ~(1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_EQUAL_INT(123, profile.outputs[1].trim);
}

static void test_autotrim_failsafe_reverts_and_requires_switch_cycle() {
  arm();
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  finish_autotrim();
  flags.failsafe_signal_lost = 1;
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[1].trim);
  flags.failsafe_signal_lost = 0;
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
}

static void test_autotrim_switch_off_on_disarm_cancels_without_saving() {
  profile_t saved = profile;
  saved.outputs[1].trim = 321;
  fmc_write_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  arm();
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  finish_autotrim();
  state.aux_active &= ~((1U << AUX_AUTOTRIM) | (1U << AUX_ARMING));
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[1].trim);
  fmc_read_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  TEST_ASSERT_EQUAL_INT(321, saved.outputs[1].trim);
}

static void test_autotrim_bounds_centers_and_excludes_throttle_mixes() {
  arm();
  pwm_motor();
  // A PWM throttle output must not be trimmed even with a roll mix attached.
  profile.mixer[6] = {.output_index = 0, .source = OUTPUT_SOURCE_ROLL, .weight = 10};
  state.rx_filtered.roll = 0.8f;
  start_autotrim();
  finish_autotrim();
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[0].trim);
  TEST_ASSERT_EQUAL_INT(500, profile.outputs[1].trim);
  TEST_ASSERT_EQUAL_INT(-500, profile.outputs[2].trim);
}

int main() {
  // Native flash writes stay in a private directory, away from simulator data.
  char original_dir[4096];
  char flash_dir[] = "/tmp/quicksilver-wing-test-XXXXXX";
  if (!getcwd(original_dir, sizeof(original_dir)) || !mkdtemp(flash_dir) || chdir(flash_dir) != 0)
    return 1;
  UNITY_BEGIN();
  RUN_TEST(test_pwm_throttle_range_and_limit);
  RUN_TEST(test_pwm_motor_stops_but_disarmed_surfaces_work);
  RUN_TEST(test_pwm_shared_rate_validation);
  RUN_TEST(test_gliding_retains_airborne_state_and_motor_cutoff);
  RUN_TEST(test_autotrim_requires_switch_and_averages_applied_surfaces_once);
  RUN_TEST(test_autotrim_captures_integral_correction_before_reset);
  RUN_TEST(test_autotrim_cancel_during_capture_and_on_early_disarm);
  RUN_TEST(test_autotrim_saves_on_disarm_with_switch_on);
  RUN_TEST(test_autotrim_failsafe_reverts_and_requires_switch_cycle);
  RUN_TEST(test_autotrim_switch_off_on_disarm_cancels_without_saving);
  RUN_TEST(test_autotrim_bounds_centers_and_excludes_throttle_mixes);
  run_wing_safety_tests();
  const int result = UNITY_END();
  unlink("flash.bin");
  chdir(original_dir);
  rmdir(flash_dir);
  return result;
}
