#include "flight/pid.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "core/profile.h"
#include "core/project.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/led.h"
#include "util/util.h"

#define PID_SIZE 3

// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f from H101 code
#define PID_VC_FACTOR 1.33f

#define RELAX_FACTOR (RELAX_FACTOR_DEG * DEGTORAD)
#define RELAX_FACTOR_YAW (RELAX_FACTOR_YAW_DEG * DEGTORAD)

// "p term setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
static const float setpoint_weigth_brushed[3] = {0.93, 0.93, 0.9}; // BRUSHED FREESTYLE
// float setpoint_weigth_brushed[3] = { 0.97 , 0.98 , 0.95};   //BRUSHED RACE
static const float setpoint_weigth_brushless[3] = {1, 1, 1}; // ALL PID

/// output limit
static const float out_limit_brushed[PID_SIZE] = {1.7, 1.7, 0.5};
static const float out_limit_brushless[PID_SIZE] = {0.8, 0.8, 0.6};

// limit of integral term (abs)
static const float integral_limit_brushed[PID_SIZE] = {1.7, 1.7, 0.5};
static const float integral_limit_brushless[PID_SIZE] = {0.8, 0.8, 0.6};

static const float pid_scales[PID_SIZE][PID_SIZE] = {
    // roll, pitch, yaw
    {1.0f / 628.0f, 1.0f / 628.0f, 1.0f / 314.0f}, // kp
    {1.0f / 50.0f, 1.0f / 50.0f, 1.0f / 50.0f},    // ki
    {1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 120.0f}, // kd
};

static float lasterror[PID_SIZE] = {0, 0, 0};
static float lasterror2[PID_SIZE] = {0, 0, 0};

static float lastrate[PID_SIZE] = {0, 0, 0};
static float lastsetpoint[PID_SIZE] = {0, 0, 0};

static float ierror[PID_SIZE] = {0, 0, 0};

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static filter_lp_pt1 dynamic_filter;
static filter_state_t dynamic_filter_state[3];

static filter_lp_pt1 rx_filter;
static filter_state_t rx_filter_state[3];

void pid_init() {
  filter_lp_pt1_init(&rx_filter, rx_filter_state, 3, state.rx_filter_hz);

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.dterm[i].type, &filter[i], filter_state[i], 3, profile.filter.dterm[i].cutoff_freq);
  }

  if (profile.filter.dterm_dynamic_enable) {
    // zero out filter, freq will be updated later on
    filter_lp_pt1_init(&dynamic_filter, dynamic_filter_state, 3, DTERM_DYNAMIC_FREQ_MAX);
  }
}

// (iwindup = 0  windup is not allowed)   (iwindup = 1 windup is allowed)
static inline float pid_compute_iterm_windup(uint8_t x, float pid_output) {
  const float *out_limit = target.brushless ? out_limit_brushless : out_limit_brushed;
  if ((pid_output >= out_limit[x]) && (state.error.axis[x] > 0)) {
    return 0.0f;
  }
  if ((pid_output <= -out_limit[x]) && (state.error.axis[x] < 0)) {
    return 0.0f;
  }

#ifdef ITERM_RELAX //  Roll - Pitch  Setpoint based I term relax method
  static float avg_setpoint[3] = {0, 0, 0};
  if (x < 2) {
    lpf(&avg_setpoint[x], state.setpoint.axis[x], FILTERCALC(state.looptime, 1.0f / (float)RELAX_FREQUENCY_HZ)); // 11 Hz filter
    const float hpfSetpoint = fabsf(state.setpoint.axis[x] - avg_setpoint[x]);
    return max(1.0f - hpfSetpoint / RELAX_FACTOR, 0.0f);
  }
#ifdef ITERM_RELAX_YAW
  else {                                                                                                             // axis is yaw
    lpf(&avg_setpoint[x], state.setpoint.axis[x], FILTERCALC(state.looptime, 1.0f / (float)RELAX_FREQUENCY_HZ_YAW)); // 25 Hz filter
    const float hpfSetpoint = fabsf(state.setpoint.axis[x] - avg_setpoint[x]);
    return max(1.0f - hpfSetpoint / RELAX_FACTOR_YAW, 0.0f);
  }
#endif
#endif

  return 1.0f;
}

static inline float pid_filter_dterm(uint8_t x, float dterm) {
  dterm = filter_step(profile.filter.dterm[0].type, &filter[0], &filter_state[0][x], dterm);
  dterm = filter_step(profile.filter.dterm[1].type, &filter[1], &filter_state[1][x], dterm);

  if (profile.filter.dterm_dynamic_enable) {
    dterm = filter_lp_pt1_step(&dynamic_filter, &dynamic_filter_state[x], dterm);
  }

  return dterm;
}

static inline bool pid_should_enable_iterm(uint8_t x) {
  static bool stick_movement = false;
  if (!flags.arm_state) {
    // disarmed, disable, flag no movement
    stick_movement = false;
    return false;
  }
  if (flags.in_air) {
    // in-air, enable, flag no movement
    stick_movement = false;
    return true;
  }

  if (fabsf(state.setpoint.axis[x]) > 0.1f) {
    // record first stick crossing
    stick_movement = true;
  }

  // enable if we recored stick movement previously
  return stick_movement;
}

static inline float pid_voltage_compensation() {
  if (!profile.voltage.pid_voltage_compensation) {
    return 1.0f;
  }

  float res = constrain(mapf((state.vbat_filtered_decay / (float)state.lipo_cell_count), 2.5f, 3.85f, PID_VC_FACTOR, 1.0f), 1.0f, PID_VC_FACTOR);
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
  // rotates errors, originally by joelucid

  // rotation around x axis:
  ierror[1] -= ierror[2] * state.gyro_delta_angle.axis[0];
  ierror[2] += ierror[1] * state.gyro_delta_angle.axis[0];

  // rotation around y axis:
  ierror[2] -= ierror[0] * state.gyro_delta_angle.axis[1];
  ierror[0] += ierror[2] * state.gyro_delta_angle.axis[1];

  // rotation around z axis:
  ierror[0] -= ierror[1] * state.gyro_delta_angle.axis[2];
  ierror[1] += ierror[0] * state.gyro_delta_angle.axis[2];

  filter_lp_pt1_coeff(&rx_filter, state.rx_filter_hz);
  filter_coeff(profile.filter.dterm[0].type, &filter[0], profile.filter.dterm[0].cutoff_freq);
  filter_coeff(profile.filter.dterm[1].type, &filter[1], profile.filter.dterm[1].cutoff_freq);

  static vec3_t pid_output = {.roll = 0, .pitch = 0, .yaw = 0};
  const float v_compensation = pid_voltage_compensation();
  const float tda_compensation = pid_tda_compensation();
  const float *out_limit = target.brushless ? out_limit_brushless : out_limit_brushed;
  const float *setpoint_weigth = target.brushless ? setpoint_weigth_brushless : setpoint_weigth_brushed;
  const float *integral_limit = target.brushless ? integral_limit_brushless : integral_limit_brushed;

  const uint8_t stick_boost_profile = rx_aux_on(AUX_STICK_BOOST_PROFILE) ? STICK_PROFILE_ON : STICK_PROFILE_OFF;
  const float *stick_accelerator = profile.pid.stick_rates[stick_boost_profile].accelerator.axis;
  const float *stick_transition = profile.pid.stick_rates[stick_boost_profile].transition.axis;

  if (profile.filter.dterm_dynamic_enable) {
    float dynamic_throttle = state.throttle + state.throttle * (1 - state.throttle) * DTERM_DYNAMIC_EXPO;
    float d_term_dynamic_freq = mapf(dynamic_throttle, 0.0f, 1.0f, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
    d_term_dynamic_freq = constrain(d_term_dynamic_freq, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);

    filter_lp_pt1_coeff(&dynamic_filter, d_term_dynamic_freq);
  }

#pragma GCC unroll 3
  for (uint8_t x = 0; x < PID_SIZE; x++) {
    const float current_kp = profile_current_pid_rates()->kp.axis[x] * pid_scales[0][x];
    const float current_ki = profile_current_pid_rates()->ki.axis[x] * pid_scales[1][x];
    const float current_kd = profile_current_pid_rates()->kd.axis[x] * pid_scales[2][x];

    // P term
    state.pid_p_term.axis[x] = state.error.axis[x] * (setpoint_weigth[x]) * current_kp;
    state.pid_p_term.axis[x] += -(1.0f - setpoint_weigth[x]) * current_kp * state.gyro.axis[x];

    // Pid Voltage Comp applied to P term only
    state.pid_p_term.axis[x] *= v_compensation;

    // I term
    const float iterm_windup = pid_compute_iterm_windup(x, pid_output.axis[x]);
    if (!pid_should_enable_iterm(x)) {
      // wind down integral while we are still on ground and we do not get any input from the sticks
      ierror[x] *= 0.98f;
    }
    // SIMPSON_RULE_INTEGRAL
    // assuming similar time intervals
    ierror[x] = ierror[x] + 0.5f * (1.0f / 3.0f) * (lasterror2[x] + 4 * lasterror[x] + state.error.axis[x]) * current_ki * iterm_windup * state.looptime;
    ierror[x] = constrain(ierror[x], -integral_limit[x], integral_limit[x]);
    lasterror2[x] = lasterror[x];
    lasterror[x] = state.error.axis[x];

    state.pid_i_term.axis[x] = ierror[x];

    // D term
    float transition_setpoint_weight = 0;
    if (stick_accelerator[x] < 1) {
      transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * stick_transition[x]) + (1 - stick_transition[x]);
    } else {
      transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * (stick_transition[x] / stick_accelerator[x])) + (1 - stick_transition[x]);
    }

    float setpoint_derivative = (state.setpoint.axis[x] - lastsetpoint[x]) * current_kd * state.timefactor;
    if (state.rx_filter_hz > 0.1f) {
      setpoint_derivative = filter_lp_pt1_step(&rx_filter, &rx_filter_state[x], setpoint_derivative);
    }
    lastsetpoint[x] = state.setpoint.axis[x];

    const float gyro_derivative = (state.gyro.axis[x] - lastrate[x]) * current_kd * state.timefactor * tda_compensation;
    lastrate[x] = state.gyro.axis[x];

    const float dterm = (setpoint_derivative * stick_accelerator[x] * transition_setpoint_weight) - (gyro_derivative);
    state.pid_d_term.axis[x] = pid_filter_dterm(x, dterm);

    state.pidoutput.axis[x] = pid_output.axis[x] = state.pid_p_term.axis[x] + state.pid_i_term.axis[x] + state.pid_d_term.axis[x];
    state.pidoutput.axis[x] = constrain(state.pidoutput.axis[x], -out_limit[x], out_limit[x]);
  }
}
