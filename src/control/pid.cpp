#include "control/pid.h"

#ifndef VEHICLE_ROVER

#include <math.h>
#include <stdbool.h>

#include "control/control.h"
#include "control/wing/navigation.h"
#include "core/profile.h"
#include "util/filter.h"
#include "util/util.h"
#include "util/vector.h"

#define PID_SIZE 3

#define PID_VC_FACTOR 1.33f
// Time constant the integral decays with while it is disabled, in seconds.
static constexpr float ITERM_DECAY_TIME = 0.05f;
#define RELAX_FACTOR (RELAX_FACTOR_DEG * DEGTORAD)
#define RELAX_FACTOR_YAW (RELAX_FACTOR_YAW_DEG * DEGTORAD)

/// output limit
#ifdef VEHICLE_WING
// Surface commands use full travel; each output's min/max limits the servo.
static constexpr vec3_t out_limit = {{1.0f, 1.0f, 1.0f}};
#else
static constexpr vec3_t out_limit = {{0.8f, 0.8f, 0.6f}};
#endif

// limit of integral term (abs)
static constexpr vec3_t integral_limit = {{0.8f, 0.8f, 0.6f}};

// PID scaling factors - pre-computed reciprocals for kp, ki, kd
// These divisions are guaranteed to be evaluated at compile-time
static constexpr vec3_t pid_scales[PID_SIZE] = {
    {{1.0f / 628.0f, 1.0f / 628.0f, 1.0f / 314.0f}},     // kp
    {{1.0f / 100.0f, 1.0f / 100.0f, 1.0f / 100.0f}},     // ki - includes historical 0.5x scaling from Silverware
    {{1.0f / 37500.0f, 1.0f / 37500.0f, 1.0f / 37500.0f}}, // kd - includes 0.0032 constant (0.0032 / 120 = 1 / 37500)
};

static vec3_t lastrate = {0};
static vec3_t lastsetpoint = {0};
static vec3_t ierror = {0};
static vec3_t last_error = {0};
static vec3_t last_error2 = {0};
static pid_rate_t scaled_rates;

static filter_t<vec3_t> filter[FILTER_MAX_SLOTS];
static filter_t<vec3_t> dynamic_filter;
#ifndef VEHICLE_WING
// Smooths the setpoint change behind the feedforward at the receiver's rate.
static filter_t<vec3_t> feedforward_filter;
// Whether the previous pass took its setpoint from the pilot.
static bool feedforward_pilot;
#endif
#ifdef ITERM_RELAX
// Slow setpoint per axis; I-term relax acts on the setpoint's distance from it.
static filter_t<> relax_filter[PID_SIZE];
#endif

void pid_reset_i(uint8_t axis) {
  ierror.axis[axis] = 0.0f;
  last_error.axis[axis] = 0.0f;
  last_error2.axis[axis] = 0.0f;
}

void pid_reset_i() {
  for (uint8_t axis = 0; axis < PID_SIZE; axis++) {
    pid_reset_i(axis);
  }
}

const vec3_t *pid_get_ierror() {
  return &ierror;
}

void pid_rates_update() {
  const pid_rate_t *rates = profile_current_pid_rates();
  scaled_rates.kp = rates->kp * pid_scales[0];
  scaled_rates.ki = rates->ki * pid_scales[1] * (1.0f / 3.0f);
  scaled_rates.kd = rates->kd * pid_scales[2];
#ifdef VEHICLE_WING
  scaled_rates.kff = rates->kff * 0.01f;
#else
  scaled_rates.kff = rates->kff * pid_scales[2];
#endif
}

void pid_filter_update(bool reset) {
  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    if (reset) {
      filter[i].init(profile.filter.dterm[i].type, profile.filter.dterm[i].cutoff_freq, state.looptime_autodetect);
    } else {
      filter[i].coeff(profile.filter.dterm[i].type, profile.filter.dterm[i].cutoff_freq, state.looptime_autodetect);
    }
  }
  if (reset) {
    dynamic_filter.init(profile.filter.dterm_dynamic_type, DTERM_DYNAMIC_FREQ_MAX, state.looptime_autodetect);
  }
#ifdef ITERM_RELAX
  for (uint8_t x = 0; x < PID_SIZE; x++) {
    const float relax_hz = x < 2 ? RELAX_FREQUENCY_HZ : RELAX_FREQUENCY_HZ_YAW;
    if (reset) {
      relax_filter[x].init(FILTER_LP_PT1, relax_hz, state.looptime_autodetect);
    } else {
      relax_filter[x].coeff(FILTER_LP_PT1, relax_hz, state.looptime_autodetect);
    }
  }
#endif
}

void pid_init() {
  pid_rates_update();
#ifndef VEHICLE_WING
  feedforward_filter.init(FILTER_LP_PT1, state.rx_filter_hz, state.looptime_autodetect);
  feedforward_pilot = false;
#endif
  pid_filter_update(true);
  lastrate = (vec3_t){0};
  lastsetpoint = (vec3_t){0};
  pid_reset_i();
}

static inline vec3_t pid_compute_iterm_windup_vec(const vec3_t *pid_output) {
  vec3_t windup = {.roll = 1.0f, .pitch = 1.0f, .yaw = 1.0f};

  for (uint8_t x = 0; x < PID_SIZE; x++) {
    if ((pid_output->axis[x] >= out_limit.axis[x]) && (state.error.axis[x] > 0)) {
      windup.axis[x] = 0.0f;
    } else if ((pid_output->axis[x] <= -out_limit.axis[x]) && (state.error.axis[x] < 0)) {
      windup.axis[x] = 0.0f;
    }
#ifdef ITERM_RELAX
    else {
      if (x < 2) {
        const float hpf_setpoint = fabsf(state.setpoint.axis[x] - relax_filter[x].step(state.setpoint.axis[x]));
        windup.axis[x] = MAX(1.0f - hpf_setpoint / RELAX_FACTOR, 0.0f);
      }
#ifdef ITERM_RELAX_YAW
      else {
        const float hpf_setpoint = fabsf(state.setpoint.axis[x] - relax_filter[x].step(state.setpoint.axis[x]));
        windup.axis[x] = MAX(1.0f - hpf_setpoint / RELAX_FACTOR_YAW, 0.0f);
      }
#endif
    }
#endif
  }

  return windup;
}

static inline vec3_t pid_filter_dterm(vec3_t dterm) {
  dterm = filter[0].step(dterm);
  dterm = filter[1].step(dterm);
  return dynamic_filter.step(dterm);
}

static inline vec3_t pid_should_enable_iterm_vec() {
  static bool stick_movement = false;
  if (!flags.arm_state) {
    stick_movement = false;
    return (vec3_t){{0.0f, 0.0f, 0.0f}};
  }
#ifdef VEHICLE_MULTI
  // A mission takeoff waits and spins up on the ground, where the resting
  // tilt reads as an angle error: integrating it tips the craft at spin-up.
  if (state.nav_state == NAV_STATE_TAKEOFF_READY || state.nav_state == NAV_STATE_TAKEOFF_SPINUP)
    return (vec3_t){{0.0f, 0.0f, 0.0f}};
#endif
  if (flags.in_air) {
    stick_movement = false;
    return (vec3_t){{1.0f, 1.0f, 1.0f}};
  }

  if (fabsf(state.setpoint.roll) > 0.1f ||
      fabsf(state.setpoint.pitch) > 0.1f ||
      fabsf(state.setpoint.yaw) > 0.1f) {
    stick_movement = true;
  }

  const float enable = stick_movement ? 1.0f : 0.0f;
  return (vec3_t){{enable, enable, enable}};
}

static inline float pid_voltage_compensation() {
  if (!profile.voltage.pid_voltage_compensation) {
    return 1.0f;
  }

  // Thrust follows the voltage under load, so use the sag filter rather than
  // the slower display filter.
  const float cell_voltage = state.vbat_sag_filtered / (float)state.lipo_cell_count;
  return constrain(mapf(cell_voltage, 2.5f, 3.85f, PID_VC_FACTOR, 1.0f), 1.0f, PID_VC_FACTOR);
}

static inline float pid_tda_compensation() {
  if (!profile.pid.throttle_dterm_attenuation.tda_active) {
    return 1.0f;
  }
  const float tda_compensation = mapf(state.throttle, profile.pid.throttle_dterm_attenuation.tda_breakpoint, 1.0f, 1.0f, profile.pid.throttle_dterm_attenuation.tda_percent);
  return constrain(tda_compensation, profile.pid.throttle_dterm_attenuation.tda_percent, 1.0f);
}

HOT_CODE void pid_calc() {
  const float dynamic_throttle = state.throttle + state.throttle * (1.0f - state.throttle);
  const float dterm_dynamic_raw_freq = mapf(dynamic_throttle, 0.0f, 1.0f, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
  const float dterm_dynamic_freq = constrain(dterm_dynamic_raw_freq, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
  dynamic_filter.coeff(profile.filter.dterm_dynamic_type, dterm_dynamic_freq, state.looptime_autodetect);

  static vec3_t pid_output = {.roll = 0, .pitch = 0, .yaw = 0};
  const float v_compensation = pid_voltage_compensation();
  const float tda_compensation = pid_tda_compensation();

  ierror = vec3_rotate(ierror, state.gyro_delta_angle);

  const vec3_t gyro_delta = state.gyro - lastrate;
#ifdef VEHICLE_WING
  // Surface moment follows dynamic pressure (V^2); the body rate a steady
  // deflection holds follows V. Gains are tuned at the cruise speed, and
  // apply as tuned on the ground and while either speed is unset.
  const float cruise_speed = profile.navigation.cruise_speed;
  const float speed_scale = state.airspeed > 0.0f && cruise_speed > 0.0f
                                ? constrain(cruise_speed / state.airspeed, WING_SPEED_SCALE_MIN, WING_SPEED_SCALE_MAX)
                                : 1.0f;
  const float pid_speed_compensation = speed_scale * speed_scale;
  // Feed the requested body rate, including level/autolaunch corrections.
  // This remains active while disarmed; the output layer gates motors.
  const vec3_t feedforward = state.setpoint * (scaled_rates.kff * speed_scale);
#else
  const float pid_speed_compensation = 1.0f;
  // Motors produce angular acceleration, so the feedforward is the setpoint's
  // rate of change. It follows the pilot's sticks only: navigation and other
  // overrides step the setpoint at their own cadence, and the jump on taking
  // over or handing back is not a stick movement.
  const bool pilot = !flags.controls_override;
  vec3_t feedforward = {};
  if (pilot && feedforward_pilot)
    feedforward = (state.setpoint - lastsetpoint) * (scaled_rates.kff * state.looptime_inverse);
  feedforward_pilot = pilot;
  if (state.rx_filter_hz > 0.1f) {
    feedforward_filter.coeff(FILTER_LP_PT1, state.rx_filter_hz, state.looptime_autodetect);
    feedforward = feedforward_filter.step(feedforward);
  }
#endif
  const vec3_t current_kp = scaled_rates.kp * (v_compensation * pid_speed_compensation);
  const vec3_t current_ki = scaled_rates.ki * (state.looptime * pid_speed_compensation);
  const vec3_t current_kd = scaled_rates.kd * (state.looptime_inverse * pid_speed_compensation);
  const vec3_t iterm_enable = pid_should_enable_iterm_vec();
  const float iterm_decay = 1.0f - state.looptime / ITERM_DECAY_TIME;
  const vec3_t iterm_windup = pid_compute_iterm_windup_vec(&pid_output);

  vec3_t dterm;
#pragma GCC unroll 3
#pragma GCC ivdep
  for (uint8_t x = 0; x < PID_SIZE; x++) {
    state.pid_p_term.axis[x] = state.error.axis[x] * current_kp.axis[x];

    const float simpson_sum = last_error2.axis[x] + 4.0f * last_error.axis[x] + state.error.axis[x];
    const float ierror_delta = simpson_sum * current_ki.axis[x] * iterm_windup.axis[x];
    ierror.axis[x] = iterm_enable.axis[x] ? (ierror.axis[x] + ierror_delta) : (ierror.axis[x] * iterm_decay);
    ierror.axis[x] = constrain(ierror.axis[x], -integral_limit.axis[x], integral_limit.axis[x]);
    state.pid_i_term.axis[x] = ierror.axis[x];

    dterm.axis[x] = -gyro_delta.axis[x] * current_kd.axis[x] * tda_compensation;
  }

  // Filter stages run across all axes; the unfiltered term stays local.
  state.pid_d_term = pid_filter_dterm(dterm);

#pragma GCC unroll 3
  for (uint8_t x = 0; x < PID_SIZE; x++) {
    pid_output.axis[x] = state.pid_p_term.axis[x] + state.pid_i_term.axis[x] + state.pid_d_term.axis[x] + feedforward.axis[x];
    state.pidoutput.axis[x] = constrain(pid_output.axis[x], -out_limit.axis[x], out_limit.axis[x]);
  }

  last_error2 = last_error;
  last_error = state.error;
  lastsetpoint = state.setpoint;
  lastrate = state.gyro;
}

#endif
