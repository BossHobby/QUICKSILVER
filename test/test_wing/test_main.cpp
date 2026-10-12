#include "mock_outputs.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unity.h>

#include "control/control.h"
#include "control/output.h"
#include "control/pid.h"
#include "control/wing/navigation.h"
#include "core/failloop.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/motor.h"
#include "driver/servo.h"
#include "driver/time.h"
#include "util/util.h"

extern const profile_t default_profile;
extern void run_wing_safety_tests();
extern void run_wing_navigation_tests();
void setUp() {
  profile = default_profile;
  profile_output_update();
  state = {};
  flags = {};
  target = {};
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

static void test_level_tilt_moves_disarmed_surfaces_without_integral() {
  pwm_motor();
  state.aux_active = 1U << AUX_LEVELMODE;
  state.GEstG.pitch = 0.5f;
  state.GEstG.yaw = 0.8660254f;
  for (int i = 0; i < 20; i++) {
    control();
  }
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.pid_i_term.pitch);
  TEST_ASSERT_LESS_THAN_FLOAT(-0.6f, pwm_values[1]);
  TEST_ASSERT_LESS_THAN_FLOAT(-0.6f, pwm_values[2]);
}
static float roll_feedforward() {
  return state.pidoutput.roll - state.pid_p_term.roll - state.pid_i_term.roll - state.pid_d_term.roll;
}
static void test_pid_gains_follow_speed_scale() {
  state.setpoint.roll = 0.1f;
  state.error.roll = 0.1f;
  state.wing_speed_scale = 1.0f;
  pid_calc();
  const float cruise_p = state.pid_p_term.roll;
  const float cruise_ff = roll_feedforward();

  // At 1 / 1.25 of cruise airspeed the surfaces have 1 / 1.5625 of the authority.
  state.wing_speed_scale = 1.25f;
  pid_calc();
  TEST_ASSERT_FLOAT_WITHIN(0.001f * cruise_p, 1.5625f * cruise_p, state.pid_p_term.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.001f * cruise_ff, 1.25f * cruise_ff, roll_feedforward());
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
static void fly_throttle(float throttle, float seconds) {
  state.rx_filtered.throttle = throttle / 1.0526316f + 0.05f;
  for (int i = 0; i < (int)(seconds / state.looptime); i++)
    control();
}
static void test_speed_scale_estimates_airspeed_from_throttle_and_climb() {
  const float cruise = profile.navigation.cruise_throttle;
  arm();
  state.looptime = 0.01f;
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.wing_speed_scale);
  fly_throttle(cruise, 1.0f);
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, state.wing_speed_scale);

  // Three quarters of the level throttle is about sqrt(3 / 4) of cruise
  // airspeed, after the airspeed has had time to settle.
  fly_throttle(cruise * 0.75f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, state.wing_speed_scale);
  fly_throttle(cruise * 0.75f, 20.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, sqrtf(4.0f / 3.0f), state.wing_speed_scale);

  // Climb throttle holds cruise airspeed; attitude pitch is positive nose-down.
  state.attitude.pitch = -10.0f * DEGTORAD;
  fly_throttle(cruise + 0.15f, 20.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, state.wing_speed_scale);

  // An idle glide is bounded by the scale limit.
  state.attitude.pitch = 0.0f;
  fly_throttle(0.0f, 20.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.25f, state.wing_speed_scale);

  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  TEST_ASSERT_FALSE(flags.arm_state);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.wing_speed_scale);
}
static vec3_t turn_setpoint_change(float roll) {
  state.attitude.roll = roll;
  state.GEstG = {{fastsin(roll), 0.0f, fastcos(roll)}};
  profile.wing.turn_coordination = 0;
  control();
  const vec3_t uncoordinated = state.setpoint;
  profile.wing.turn_coordination = 1;
  control();
  return vec3_sub(state.setpoint, uncoordinated);
}
static void test_level_turn_coordination_adds_turn_rates() {
  arm();
  fly_throttle(profile.navigation.cruise_throttle, 0.01f);
  state.aux_active |= 1U << AUX_LEVELMODE;

  // g * tan(30 deg) / 15 m/s about earth-up: nose up and yaw into the turn.
  const float turn_rate = 9.80665f * tanf(30.0f * DEGTORAD) / profile.navigation.cruise_speed;
  const vec3_t right = turn_setpoint_change(30.0f * DEGTORAD);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, right.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, -turn_rate * 0.5f, right.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, turn_rate * 0.8660254f, right.yaw);

  const vec3_t left = turn_setpoint_change(-30.0f * DEGTORAD);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, -turn_rate * 0.5f, left.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, -turn_rate * 0.8660254f, left.yaw);

  // At 1 / 1.2 of cruise airspeed the same bank turns 1.2 times faster.
  state.looptime = 0.01f;
  fly_throttle(profile.navigation.cruise_throttle / 1.44f, 20.0f);
  const vec3_t slow = turn_setpoint_change(30.0f * DEGTORAD);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.2f * right.pitch, slow.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.2f * right.yaw, slow.yaw);

  // Beyond the fade the turn is no longer a coordinated level turn.
  const vec3_t knife_edge = turn_setpoint_change(85.0f * DEGTORAD);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, knife_edge.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, knife_edge.yaw);
}
static void test_turn_coordination_waits_for_flight() {
  state.aux_active = 1U << AUX_LEVELMODE;
  const vec3_t change = turn_setpoint_change(30.0f * DEGTORAD);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, change.pitch);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, change.yaw);
}
static void test_navigation_always_coordinates_turns() {
  arm();
  fly_throttle(profile.navigation.cruise_throttle, 0.01f);
  profile.wing.turn_coordination = 0;
  state.wing_nav_state = WING_NAV_LOITER;
  wing_nav_command = {.roll = 30.0f * DEGTORAD, .pitch = 0.0f, .throttle = profile.navigation.cruise_throttle};
  state.attitude.roll = 30.0f * DEGTORAD;
  state.GEstG = {{0.5f, 0.0f, 0.8660254f}};
  control();
  const float turn_rate = 9.80665f * tanf(30.0f * DEGTORAD) / profile.navigation.cruise_speed;
  TEST_ASSERT_FLOAT_WITHIN(0.002f, turn_rate * 0.8660254f, state.setpoint.yaw);
}
// Fly at cruise throttle for the given time at a constant bank and airspeed
// in a steady wind, with 10 Hz GPS velocity.
static float gps_glitch_north = 0.0f; // added to the next GPS fix only
static void fly_gps(float bank, float airspeed, float wind_north, float wind_east, float seconds) {
  const float turn_rate = bank == 0.0f ? 0.0f : 9.80665f * tanf(bank) / airspeed;
  // Heading from elapsed time rather than summed steps, wrapped between calls,
  // so float resolution does not bias the simulated turn.
  static float start_heading = 0.0f;
  const int loops = (int)(seconds / state.looptime);
  state.attitude.roll = bank;
  state.GEstG = {{sinf(bank), 0.0f, cosf(bank)}};
  state.gyro = {{0.0f, -turn_rate * sinf(bank), turn_rate * cosf(bank)}};
  state.gps_lock = true;
  state.rx_filtered.throttle = profile.navigation.cruise_throttle / 1.0526316f + 0.05f;
  for (int i = 0; i < loops; i++) {
    time_test_advance_us((uint32_t)(state.looptime * 1e6f));
    if (i % (int)(0.1f / state.looptime) == 0) {
      const float heading = start_heading + turn_rate * (i + 1) * state.looptime;
      state.gps_vel_north = airspeed * cosf(heading) + wind_north + gps_glitch_north;
      gps_glitch_north = 0.0f;
      state.gps_vel_east = airspeed * sinf(heading) + wind_east;
      state.gps_speed = hypotf(state.gps_vel_north, state.gps_vel_east);
      state.gps_last_update_ms = time_millis();
    }
    control();
  }
  start_heading = fmodf(start_heading + turn_rate * loops * state.looptime, 2.0f * M_PI_F);
}
static void test_cruise_speed_learned_from_turns_in_wind_and_saved_on_disarm() {
  state.looptime = 0.002f;
  arm();
  TEST_ASSERT_EQUAL_FLOAT(15.0f, profile.navigation.cruise_speed);

  // Straight flight in wind shows ground speed only and teaches nothing.
  fly_gps(0.0f, 20.0f, 6.0f, -4.0f, 30.0f);
  TEST_ASSERT_TRUE(flags.in_air);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, profile.navigation.cruise_speed);

  // Turns measure airspeed; the wind cancels across each heading change.
  fly_gps(30.0f * DEGTORAD, 20.0f, 6.0f, -4.0f, 120.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 20.0f, profile.navigation.cruise_speed);

  profile_t saved;
  fmc_read_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  TEST_ASSERT_NOT_EQUAL_FLOAT(profile.navigation.cruise_speed, saved.navigation.cruise_speed);

  // Landed and disarmed: the learned speed is stored once the wing stops.
  state.gps_speed = 0.0f;
  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  control();
  fmc_read_buf(PROFILE_STORAGE_OFFSET + FMC_MAGIC_SIZE, (uint8_t *)&saved, sizeof(saved));
  TEST_ASSERT_EQUAL_FLOAT(profile.navigation.cruise_speed, saved.navigation.cruise_speed);
}
static constexpr float KPH = 1.0f / 3.6f;
static void test_cruise_speed_learns_fast_wing_within_two_minutes() {
  state.looptime = 0.002f;
  arm();
  // 126 km/h from the 54 km/h default, in gentle 15 degree turns.
  fly_gps(15.0f * DEGTORAD, 126.0f * KPH, 4.0f, 2.0f, 120.0f);
  TEST_ASSERT_FLOAT_WITHIN(2.0f * KPH, 126.0f * KPH, profile.navigation.cruise_speed);
}
static void test_cruise_speed_learns_slow_wing_within_half_a_minute() {
  state.looptime = 0.002f;
  arm();
  fly_gps(30.0f * DEGTORAD, 30.0f * KPH, 1.0f, -1.0f, 30.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f * KPH, 30.0f * KPH, profile.navigation.cruise_speed);
}
static void test_cruise_speed_learns_only_near_cruise() {
  state.looptime = 0.002f;
  arm();
  // Climbing steeply at cruise throttle, the throttle model pins its speed
  // estimate at the slow limit and cannot convert airspeed to cruise speed.
  state.attitude.pitch = -30.0f * DEGTORAD;
  fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 0.0f, 60.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.25f, state.wing_speed_scale);
  TEST_ASSERT_EQUAL_FLOAT(15.0f, profile.navigation.cruise_speed);

  state.attitude.pitch = 0.0f;
  fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 0.0f, 60.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, profile.navigation.cruise_speed);
}
static void test_cruise_speed_stays_accurate_through_long_loiter() {
  state.looptime = 0.002f;
  arm();
  // Ten minutes of circling turns the plane through about a hundred circles.
  fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 1.0f, 600.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 20.0f, profile.navigation.cruise_speed);
}
static void test_cruise_speed_ignores_gps_glitch_and_stays_in_range() {
  state.looptime = 0.002f;
  arm();
  fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 0.0f, 60.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, profile.navigation.cruise_speed);

  // One corrupted fix spoils at most the two turns that share it; the median
  // of five keeps both away from the learned value.
  for (int i = 0; i < 20; i++) {
    gps_glitch_north = 15.0f;
    fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 0.0f, 0.5f);
    fly_gps(30.0f * DEGTORAD, 20.0f, 3.0f, 0.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, profile.navigation.cruise_speed);
  }

  // The learned value stays within the range a user can set, 180 km/h.
  fly_gps(30.0f * DEGTORAD, 70.0f, 0.0f, 0.0f, 60.0f);
  TEST_ASSERT_EQUAL_FLOAT(50.0f, profile.navigation.cruise_speed);
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
// Without GPS the switch must be turned on in the air.
static void start_autotrim() {
  flags.in_air = 1;
  state.aux_active &= ~(1U << AUX_AUTOTRIM);
  control();
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
  // The switch is still on, but the next arm waits for it to be cycled.
  state.aux_active |= 1U << AUX_ARMING;
  control();
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_IDLE, state.wing_autotrim_state);
  start_autotrim();
}

static void test_autotrim_saves_on_disarm_with_switch_on() {
  arm();
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  finish_autotrim();
  state.aux_active &= ~(1U << AUX_ARMING);
  control();
  // The disarming pass started armed, without configuration ownership.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVE_PENDING, state.wing_autotrim_state);
  state.gps_lock = 1;
  state.gps_speed = 10.0f; // A disarmed wing still gliding defers the save.
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVE_PENDING, state.wing_autotrim_state);
  state.gps_speed = 0.0f;
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

static void test_autotrim_waits_for_steady_flight() {
  arm();
  state.gps_lock = 1;
  state.gps_speed = 10.0f;
  state.rx_filtered.roll = 0.2f;
  state.aux_active |= (1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
  time_test_advance_us(3000000);
  control(); // Armed on the ground: no capture.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);

  flags.in_air = 1;
  state.gyro.pitch = 1.0f;
  control();
  time_test_advance_us(3000000);
  control(); // Rotating: no capture.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);

  state.gyro.pitch = 0.0f;
  control();
  time_test_advance_us(1999000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
  time_test_advance_us(1000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_SAVE_PENDING, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(200, profile.outputs[1].trim);
}

static void test_autotrim_leaves_channel_mixes_out_of_trim() {
  arm();
  // Flaperon with flaps up at -30%, cancelled by the existing trim.
  profile.mixer[6] = {.output_index = 1, .source = OUTPUT_SOURCE_RX_CHANNEL, .source_index = 5, .weight = 30};
  state.rx_channels[5] = 0;
  profile.outputs[1].trim = 300;
  state.rx_filtered.roll = 0.2f;
  start_autotrim();
  const float held = pwm_values[1];
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, held);
  finish_autotrim();
  TEST_ASSERT_EQUAL_INT(500, profile.outputs[1].trim);

  // With the sticks released the flaps still need their offset.
  state.rx_filtered = {};
  control();
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, held, pwm_values[1]);
}

static void test_autotrim_ignores_turns_and_stabilized_stick_input() {
  arm();
  state.aux_active |= 1U << AUX_ACROMODE;
  start_autotrim();

  state.gyro.yaw = 25.0f * DEGTORAD;
  control();
  time_test_advance_us(3000000);
  control(); // Turning: no capture.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);

  state.gyro.yaw = 0.0f;
  state.rx_filtered.roll = 0.2f;
  control();
  time_test_advance_us(3000000);
  control(); // Acro stick input: no capture.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);

  state.rx_filtered.roll = 0.0f;
  control();
  finish_autotrim();
}

// Armed with the switch already on, then a throttle run-up that latches in_air.
static void run_up_with_autotrim_on() {
  state.aux_active = 1U << AUX_PREARM;
  control(); // Disarmed with the switch off, as at boot.
  state.aux_active = (1U << AUX_ARMING) | (1U << AUX_PREARM) | (1U << AUX_AUTOTRIM);
  control();
  TEST_ASSERT_TRUE(flags.arm_state);
  state.rx_filtered.throttle = 0.5f;
  state.rx_filtered.roll = 0.2f;
  control();
  TEST_ASSERT_TRUE(flags.in_air);
}

static void test_autotrim_run_up_needs_flight_evidence() {
  run_up_with_autotrim_on();
  time_test_advance_us(3000000);
  control(); // A run-up is not flight.
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
  TEST_ASSERT_EQUAL_INT(0, profile.outputs[1].trim);

  // Neither is a GPS fix while standing.
  state.gps_lock = 1;
  control();
  time_test_advance_us(3000000);
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);
  state.gps_lock = 0;

  // Turning the switch on in the air is the pilot's evidence.
  start_autotrim();
  finish_autotrim();
  TEST_ASSERT_EQUAL_INT(200, profile.outputs[1].trim);
}

static void test_autotrim_climb_shows_flight_without_gps() {
  run_up_with_autotrim_on();
  state.baro_valid = 1;
  state.altitude = 10.0f;
  control();
  // Descending back toward launch height keeps the flight latched.
  state.altitude = 1.0f;
  finish_autotrim();
  TEST_ASSERT_EQUAL_INT(200, profile.outputs[1].trim);
}

static void test_autotrim_captures_slow_nose_high_flight() {
  arm();
  state.rx_filtered.roll = 0.2f;
  state.aux_active |= 1U << AUX_AUTOTRIM;
  flags.in_air = 1;
  state.gps_lock = 1;
  state.gps_speed = 10.0f;
  control();
  TEST_ASSERT_EQUAL(WING_AUTOTRIM_ACTIVE, state.wing_autotrim_state);

  // Hanging nose-high into a headwind: slow over the ground and far from the
  // accelerometer's level, which may itself be miscalibrated.
  state.gps_speed = 2.0f;
  state.attitude.pitch = -25.0f * DEGTORAD;
  state.attitude.roll = 12.0f * DEGTORAD;
  finish_autotrim();
  TEST_ASSERT_EQUAL_INT(200, profile.outputs[1].trim);
}

static void test_failsafe_stops_surfaces_at_trimmed_center() {
  pwm_motor();
  profile.outputs[1].trim = 100;
  profile.outputs[2].invert = true;
  profile.outputs[2].trim = 50;
  profile.outputs[3].trim = 300;
  profile.outputs[3].max = 200;
  flags.failsafe_outputs_blocked = 1;
  write_throttle(1.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pwm_values[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, pwm_values[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.05f, pwm_values[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, pwm_values[3]);
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
  RUN_TEST(test_level_tilt_moves_disarmed_surfaces_without_integral);
  RUN_TEST(test_pid_gains_follow_speed_scale);
  RUN_TEST(test_speed_scale_estimates_airspeed_from_throttle_and_climb);
  RUN_TEST(test_level_turn_coordination_adds_turn_rates);
  RUN_TEST(test_turn_coordination_waits_for_flight);
  RUN_TEST(test_navigation_always_coordinates_turns);
  RUN_TEST(test_cruise_speed_learned_from_turns_in_wind_and_saved_on_disarm);
  RUN_TEST(test_cruise_speed_learns_fast_wing_within_two_minutes);
  RUN_TEST(test_cruise_speed_learns_slow_wing_within_half_a_minute);
  RUN_TEST(test_cruise_speed_learns_only_near_cruise);
  RUN_TEST(test_cruise_speed_stays_accurate_through_long_loiter);
  RUN_TEST(test_cruise_speed_ignores_gps_glitch_and_stays_in_range);
  RUN_TEST(test_pwm_shared_rate_validation);
  RUN_TEST(test_gliding_retains_airborne_state_and_motor_cutoff);
  RUN_TEST(test_autotrim_requires_switch_and_averages_applied_surfaces_once);
  RUN_TEST(test_autotrim_captures_integral_correction_before_reset);
  RUN_TEST(test_autotrim_cancel_during_capture_and_on_early_disarm);
  RUN_TEST(test_autotrim_saves_on_disarm_with_switch_on);
  RUN_TEST(test_autotrim_failsafe_reverts_and_requires_switch_cycle);
  RUN_TEST(test_autotrim_switch_off_on_disarm_cancels_without_saving);
  RUN_TEST(test_autotrim_bounds_centers_and_excludes_throttle_mixes);
  RUN_TEST(test_autotrim_waits_for_steady_flight);
  RUN_TEST(test_autotrim_leaves_channel_mixes_out_of_trim);
  RUN_TEST(test_autotrim_ignores_turns_and_stabilized_stick_input);
  RUN_TEST(test_autotrim_run_up_needs_flight_evidence);
  RUN_TEST(test_autotrim_climb_shows_flight_without_gps);
  RUN_TEST(test_autotrim_captures_slow_nose_high_flight);
  RUN_TEST(test_failsafe_stops_surfaces_at_trimmed_center);
  run_wing_safety_tests();
  run_wing_navigation_tests();
  const int result = UNITY_END();
  unlink("flash.bin");
  chdir(original_dir);
  rmdir(flash_dir);
  return result;
}
