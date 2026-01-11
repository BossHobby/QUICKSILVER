#include "flight/pid.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "core/profile.h"
#include "core/project.h"
#include "core/tasks.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/led.h"
#include "util/util.h"
#include "util/vector.h"

#define PID_SIZE 3

// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f from H101 code
#define PID_VC_FACTOR 1.33f

#define RELAX_FACTOR (RELAX_FACTOR_DEG * DEGTORAD)
#define RELAX_FACTOR_YAW (RELAX_FACTOR_YAW_DEG * DEGTORAD)

/// output limit
static const vec3_t out_limit = {.roll = 0.8f, .pitch = 0.8f, .yaw = 0.6f};

// limit of integral term (abs)
static const vec3_t integral_limit = {.roll = 0.8f, .pitch = 0.8f, .yaw = 0.6f};

static const vec3_t pid_scales[PID_SIZE] = {
    {.roll = 1.0f / 628.0f, .pitch = 1.0f / 628.0f, .yaw = 1.0f / 314.0f}, // kp
    {.roll = 1.0f / 100.0f, .pitch = 1.0f / 100.0f, .yaw = 1.0f / 100.0f}, // ki - includes historical 0.5x scaling from Silverware
    {.roll = 1.0f / 37500.0f, .pitch = 1.0f / 37500.0f, .yaw = 1.0f / 37500.0f},    // kd - includes 0.0032 constant (0.0032 / 120 = 1 / 37500)
};

static vec3_t lastrate = {.roll = 0, .pitch = 0, .yaw = 0};
static vec3_t lastsetpoint = {.roll = 0, .pitch = 0, .yaw = 0};

static vec3_t ierror;
static vec3_t last_error;
static vec3_t last_error2;

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static filter_t dynamic_filter;
static filter_state_t dynamic_filter_state[3];

static filter_lp_pt1 rx_filter;
static filter_state_t rx_filter_state[3];

void pid_init() {
  filter_lp_pt1_init(&rx_filter, rx_filter_state, 3, state.rx_filter_hz, task_get_period_us(TASK_PID));
  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.dterm[i].type, &filter[i], filter_state[i], 3, profile.filter.dterm[i].cutoff_freq, task_get_period_us(TASK_PID));
  }
  filter_init(profile.filter.dterm_dynamic_type, &dynamic_filter, dynamic_filter_state, 3, DTERM_DYNAMIC_FREQ_MAX, task_get_period_us(TASK_PID));
}

// (iwindup = 0  windup is not allowed)   (iwindup = 1 windup is allowed)
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
      static vec3_t avg_setpoint = {.roll = 0, .pitch = 0, .yaw = 0};
      static float lpf_coeff = 0;
      static float lpf_coeff_yaw = 0;
      if (lpf_coeff == 0) {
        lpf_coeff = lpfcalc(state.looptime, 1.0f / (float)RELAX_FREQUENCY_HZ);
        lpf_coeff_yaw = lpfcalc(state.looptime, 1.0f / (float)RELAX_FREQUENCY_HZ_YAW);
      }
      if (x < 2) {
        lpf(&avg_setpoint.axis[x], state.setpoint.axis[x], lpf_coeff);
        const float hpfSetpoint = fabsf(state.setpoint.axis[x] - avg_setpoint.axis[x]);
        windup.axis[x] = MAX(1.0f - hpfSetpoint / RELAX_FACTOR, 0.0f);
      }
#ifdef ITERM_RELAX_YAW
      else {
        lpf(&avg_setpoint.axis[x], state.setpoint.axis[x], lpf_coeff_yaw);
        const float hpfSetpoint = fabsf(state.setpoint.axis[x] - avg_setpoint.axis[x]);
        windup.axis[x] = MAX(1.0f - hpfSetpoint / RELAX_FACTOR_YAW, 0.0f);
      }
#endif
    }
#endif
  }

  return windup;
}

static inline float pid_filter_dterm(uint8_t x, float dterm) {
  dterm = filter_step(profile.filter.dterm[0].type, &filter[0], &filter_state[0][x], dterm);
  dterm = filter_step(profile.filter.dterm[1].type, &filter[1], &filter_state[1][x], dterm);
  dterm = filter_step(profile.filter.dterm_dynamic_type, &dynamic_filter, &dynamic_filter_state[x], dterm);
  return dterm;
}

static inline vec3_t pid_should_enable_iterm_vec() {
  static bool stick_movement = false;
  if (!flags.arm_state) {
    // disarmed, disable, flag no movement
    stick_movement = false;
    return (vec3_t){{0.0f, 0.0f, 0.0f}};
  }
  if (flags.in_air) {
    // in-air, enable, flag no movement
    stick_movement = false;
    return (vec3_t){{1.0f, 1.0f, 1.0f}};
  }

  // Check for stick movement on any axis
  if (fabsf(state.setpoint.roll) > 0.1f ||
      fabsf(state.setpoint.pitch) > 0.1f ||
      fabsf(state.setpoint.yaw) > 0.1f) {
    stick_movement = true;
  }

  // enable if we recorded stick movement previously
  const float enable = stick_movement ? 1.0f : 0.0f;
  return (vec3_t){{enable, enable, enable}};
}

static inline float pid_voltage_compensation() {
  if (!profile.voltage.pid_voltage_compensation) {
    return 1.0f;
  }

  float res = constrain(mapf(state.vbat_cell_avg, 2.5f, 3.85f, PID_VC_FACTOR, 1.0f), 1.0f, PID_VC_FACTOR);
#ifdef LEVELMODE_PID_ATTENUATION
  if (rx_aux_on(AUX_LEVELMODE))
    res *= LEVELMODE_PID_ATTENUATION;
#endif
  return res;
}

static inline float pid_tda_compensation() {
  if (!profile.pid.throttle_dterm_attenuation.tda_active) {
    return 1.0f;
  }
  const float tda_compensation = mapf(state.throttle, profile.pid.throttle_dterm_attenuation.tda_breakpoint, 1.0f, 1.0f, profile.pid.throttle_dterm_attenuation.tda_percent);
  return constrain(tda_compensation, profile.pid.throttle_dterm_attenuation.tda_percent, 1.0f);
}

// input: error[] = setpoint - gyro
// output: state.pidoutput.axis[] = change required from motors
void pid_calc() {
  filter_lp_pt1_coeff(&rx_filter, state.rx_filter_hz, task_get_period_us(TASK_PID));

  filter_coeff(profile.filter.dterm[0].type, &filter[0], profile.filter.dterm[0].cutoff_freq, task_get_period_us(TASK_PID));
  filter_coeff(profile.filter.dterm[1].type, &filter[1], profile.filter.dterm[1].cutoff_freq, task_get_period_us(TASK_PID));

  const float dynamic_throttle = state.throttle + state.throttle * (1.0f - state.throttle);
  const float dterm_dynamic_raw_freq = mapf(dynamic_throttle, 0.0f, 1.0f, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
  const float dterm_dynamic_freq = constrain(dterm_dynamic_raw_freq, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
  filter_coeff(profile.filter.dterm_dynamic_type, &dynamic_filter, dterm_dynamic_freq, task_get_period_us(TASK_PID));

  static vec3_t pid_output = {.roll = 0, .pitch = 0, .yaw = 0};
  const float v_compensation = pid_voltage_compensation();
  const float tda_compensation = pid_tda_compensation();

  const uint8_t stick_boost_profile = rx_aux_on(AUX_STICK_BOOST_PROFILE) ? STICK_PROFILE_ON : STICK_PROFILE_OFF;
  const float *stick_accelerator = profile.pid.stick_rates[stick_boost_profile].accelerator.axis;
  const float *stick_transition = profile.pid.stick_rates[stick_boost_profile].transition.axis;

  // rotates errors
  ierror = vec3_rotate(ierror, state.gyro_delta_angle);

  // Calculate deltas for derivatives
  const vec3_t setpoint_delta = vec3_sub(state.setpoint, lastsetpoint);
  const vec3_t gyro_delta = vec3_sub(state.gyro, lastrate);

  // Pre-calculate all PID gains using vec3 operations
  const pid_rate_t *rates = profile_current_pid_rates();
  const vec3_t current_kp = vec3_mul(vec3_mul_elem(rates->kp, pid_scales[0]), v_compensation);

  // Pre-calculate common terms
  const float ki_looptime = state.looptime * (1.0f / 3.0f); // Simpson's rule constant * looptime

  // Pre-multiply Ki and Kd with their time factors
  const vec3_t current_ki = vec3_mul(vec3_mul_elem(rates->ki, pid_scales[1]), ki_looptime);
  const vec3_t current_kd = vec3_mul(vec3_mul_elem(rates->kd, pid_scales[2]), state.looptime_inverse);
  const vec3_t iterm_enable = pid_should_enable_iterm_vec();
  const vec3_t iterm_windup = pid_compute_iterm_windup_vec(&pid_output);
  const bool rx_filter_enabled = state.rx_filter_hz > 0.1f;

#pragma GCC unroll 3
#pragma GCC ivdep // no loop dependencies, safe to vectorize
  for (uint8_t x = 0; x < PID_SIZE; x++) {
    // P term (voltage compensation already applied to current_kp)
    state.pid_p_term.axis[x] = state.error.axis[x] * current_kp.axis[x];

    // I term - combine decay and simpson integration
    const float simpson_sum = last_error2.axis[x] + 4.0f * last_error.axis[x] + state.error.axis[x];
    const float ierror_delta = simpson_sum * current_ki.axis[x] * iterm_windup.axis[x];
    // If iterm disabled, decay by 0.98, otherwise add the delta
    ierror.axis[x] = iterm_enable.axis[x] ? (ierror.axis[x] + ierror_delta) : (ierror.axis[x] * 0.98f);
    ierror.axis[x] = constrain(ierror.axis[x], -integral_limit.axis[x], integral_limit.axis[x]);

    state.pid_i_term.axis[x] = ierror.axis[x];

    // D term
    float transition_setpoint_weight = 0;
    if (stick_accelerator[x] < 1) {
      transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * stick_transition[x]) + (1 - stick_transition[x]);
    } else {
      if (stick_accelerator[x] > 0) {
        transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * (stick_transition[x] / stick_accelerator[x])) + (1 - stick_transition[x]);
      } else {
        transition_setpoint_weight = 1.0f;
      }
    }

    float setpoint_derivative = setpoint_delta.axis[x] * current_kd.axis[x];
    if (rx_filter_enabled) {
      setpoint_derivative = filter_lp_pt1_step(&rx_filter, &rx_filter_state[x], setpoint_derivative);
    }

    const float gyro_derivative = gyro_delta.axis[x] * current_kd.axis[x] * tda_compensation;

    const float dterm = (setpoint_derivative * stick_accelerator[x] * transition_setpoint_weight) - (gyro_derivative);
    state.pid_d_term.axis[x] = pid_filter_dterm(x, dterm);

    state.pidoutput.axis[x] = pid_output.axis[x] = state.pid_p_term.axis[x] + state.pid_i_term.axis[x] + state.pid_d_term.axis[x];
    state.pidoutput.axis[x] = constrain(state.pidoutput.axis[x], -out_limit.axis[x], out_limit.axis[x]);
  }

  // Update history
  last_error2 = last_error;
  last_error = state.error;
  lastsetpoint = state.setpoint;
  lastrate = state.gyro;
}
