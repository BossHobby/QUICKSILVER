#include "flight/pid.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "core/profile.h"
#include "core/project.h"
#include "flight/control.h"
#include "flight/filter.h"
#include "io/led.h"
#include "rx/rx.h"
#include "util/util.h"

#define PID_SIZE 3

// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f from H101 code
#define PID_VC_FACTOR 1.33f

#define RELAX_FACTOR (RELAX_FACTOR_DEG * DEGTORAD)
#define RELAX_FACTOR_YAW (RELAX_FACTOR_YAW_DEG * DEGTORAD)

extern profile_t profile;

float timefactor;

int number_of_increments[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
int current_pid_axis = 0;
int current_pid_term = 0;

// "p term setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
static const float setpoint_weigth_brushed[3] = {0.93, 0.93, 0.9}; // BRUSHED FREESTYLE
// float setpoint_weigth_brushed[3] = { 0.97 , 0.98 , 0.95};   //BRUSHED RACE
static const float setpoint_weigth_brushless[3] = {1, 1, 1}; // ALL PID

/// output limit
static const float out_limit_brushed[PID_SIZE] = {1.7, 1.7, 0.5};
static const float out_limit_brushless[PID_SIZE] = {1.0, 1.0, 1.0};

// limit of integral term (abs)
static const float integral_limit_brushed[PID_SIZE] = {1.7, 1.7, 0.5};
static const float integral_limit_brushless[PID_SIZE] = {0.8, 0.8, 0.6};

static const float pid_scales[PID_SIZE][PID_SIZE] = {
    // roll, pitch, yaw
    {628.0f, 628.0f, 314.0f}, // kp
    {50.0f, 50.0f, 50.0f},    // ki
    {120.0f, 120.0f, 120.0f}, // kd
};

static float lasterror[PID_SIZE] = {0, 0, 0};
static float lasterror2[PID_SIZE] = {0, 0, 0};

static float lastrate[PID_SIZE] = {0, 0, 0};
static float lastsetpoint[PID_SIZE] = {0, 0, 0};

static float current_kp[PID_SIZE] = {0, 0, 0};
static float current_ki[PID_SIZE] = {0, 0, 0};
static float current_kd[PID_SIZE] = {0, 0, 0};

static float ierror[PID_SIZE] = {0, 0, 0};

static float v_compensation = 1.00;
static float tda_compensation = 1.00;

static filter_t filter[FILTER_MAX_SLOTS];
static filter_state_t filter_state[FILTER_MAX_SLOTS][3];

static filter_lp_pt1 dynamic_filter;
static filter_state_t dynamic_filter_state[3];

static filter_lp_pt1 rx_filter;
static filter_state_t rx_filter_state[3];

void pid_init() {
  filter_lp_pt1_init(&rx_filter, rx_filter_state, 3, rx_smoothing_hz());

  for (uint8_t i = 0; i < FILTER_MAX_SLOTS; i++) {
    filter_init(profile.filter.dterm[i].type, &filter[i], filter_state[i], 3, profile.filter.dterm[i].cutoff_freq);
  }

  if (profile.filter.dterm_dynamic_enable) {
    // zero out filter, freq will be updated later on
    filter_lp_pt1_init(&dynamic_filter, dynamic_filter_state, 3, DYNAMIC_FREQ_MAX);
  }
}

// calculate change from ideal loop time
// 0.0032f is there for legacy purposes, should be 0.001f = looptime
// this is called in advance as an optimization because it has division
void pid_precalc() {
  timefactor = 0.0032f / state.looptime;

  filter_lp_pt1_coeff(&rx_filter, rx_smoothing_hz());
  filter_coeff(profile.filter.dterm[0].type, &filter[0], profile.filter.dterm[0].cutoff_freq);
  filter_coeff(profile.filter.dterm[1].type, &filter[1], profile.filter.dterm[1].cutoff_freq);

  if (profile.voltage.pid_voltage_compensation) {
    v_compensation = mapf((state.vbat_filtered_decay / (float)state.lipo_cell_count), 2.5f, 3.85f, PID_VC_FACTOR, 1.0f);
    v_compensation = constrainf(v_compensation, 1.0f, PID_VC_FACTOR);

#ifdef LEVELMODE_PID_ATTENUATION
    if (rx_aux_on(AUX_LEVELMODE))
      v_compensation *= LEVELMODE_PID_ATTENUATION;
#endif
  }

  if (profile.pid.throttle_dterm_attenuation.tda_active) {
    tda_compensation = mapf(state.throttle, profile.pid.throttle_dterm_attenuation.tda_breakpoint, 1.0f, 1.0f, profile.pid.throttle_dterm_attenuation.tda_percent);
    tda_compensation = constrainf(tda_compensation, profile.pid.throttle_dterm_attenuation.tda_percent, 1.0f);
  } else {
    tda_compensation = 1.0f;
  }

  if (profile.filter.dterm_dynamic_enable) {
    float dynamic_throttle = state.throttle * (1 - state.throttle / 2.0f) * 2.0f;
    float d_term_dynamic_freq = mapf(dynamic_throttle, 0.0f, 1.0f, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);
    d_term_dynamic_freq = constrainf(d_term_dynamic_freq, profile.filter.dterm_dynamic_min, profile.filter.dterm_dynamic_max);

    filter_lp_pt1_coeff(&dynamic_filter, d_term_dynamic_freq);
  }

  for (uint8_t i = 0; i < PID_SIZE; i++) {
    current_kp[i] = profile_current_pid_rates()->kp.axis[i] / pid_scales[0][i];
    current_ki[i] = profile_current_pid_rates()->ki.axis[i] / pid_scales[1][i];
    current_kd[i] = profile_current_pid_rates()->kd.axis[i] / pid_scales[2][i];
  }
}

// (iwindup = 0  windup is not allowed)   (iwindup = 1 windup is allowed)
static float pid_compute_iterm_windup(uint8_t x, float pid_output) {
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

static float pid_filter_dterm(uint8_t x, float dterm) {
  dterm = filter_step(profile.filter.dterm[0].type, &filter[0], &filter_state[0][x], dterm);
  dterm = filter_step(profile.filter.dterm[1].type, &filter[1], &filter_state[1][x], dterm);

  if (profile.filter.dterm_dynamic_enable) {
    dterm = filter_lp_pt1_step(&dynamic_filter, &dynamic_filter_state[x], dterm);
  }

  return dterm;
}

// pid calculation for acro ( rate ) mode
// input: error[x] = setpoint - gyro
// output: state.pidoutput.axis[x] = change required from motors
static void pid(uint8_t x) {
  // P term
  const float *setpoint_weigth = target.brushless ? setpoint_weigth_brushless : setpoint_weigth_brushed;
  state.pid_p_term.axis[x] = state.error.axis[x] * (setpoint_weigth[x]) * current_kp[x];
  state.pid_p_term.axis[x] += -(1.0f - setpoint_weigth[x]) * current_kp[x] * state.gyro.axis[x];

  // Pid Voltage Comp applied to P term only
  if (profile.voltage.pid_voltage_compensation) {
    state.pid_p_term.axis[x] *= v_compensation;
  }

  // I term
  // in level mode or horizon but not racemode and while on the ground...
  if ((rx_aux_on(AUX_LEVELMODE)) && (!rx_aux_on(AUX_RACEMODE)) && ((flags.on_ground) || (flags.in_air == 0))) {
    // wind down the integral error
    ierror[x] *= 0.98f;
  } else if (flags.on_ground) {
    // in acro mode - only wind down integral when idle up is off and throttle is 0
    ierror[x] *= 0.98f;
  }

  static vec3_t pid_output = {.roll = 0, .pitch = 0, .yaw = 0};

  // SIMPSON_RULE_INTEGRAL
  // assuming similar time intervals
  const float *integral_limit = target.brushless ? integral_limit_brushless : integral_limit_brushed;
  const float iterm_windup = pid_compute_iterm_windup(x, pid_output.axis[x]);
  ierror[x] = ierror[x] + 0.166666f * (lasterror2[x] + 4 * lasterror[x] + state.error.axis[x]) * current_ki[x] * iterm_windup * state.looptime;
  lasterror2[x] = lasterror[x];
  lasterror[x] = state.error.axis[x];
  limitf(&ierror[x], integral_limit[x]);

  state.pid_i_term.axis[x] = ierror[x];

  // D term
  const uint8_t stck_boost_profile = rx_aux_on(AUX_STICK_BOOST_PROFILE) ? STICK_PROFILE_ON : STICK_PROFILE_OFF;

  const float stick_accelerator = profile.pid.stick_rates[stck_boost_profile].accelerator.axis[x];
  const float stick_transition = profile.pid.stick_rates[stck_boost_profile].transition.axis[x];

  float transition_setpoint_weight = 0;
  if (stick_accelerator < 1) {
    transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * stick_transition) + (1 - stick_transition);
  } else {
    transition_setpoint_weight = (fabsf(state.rx_filtered.axis[x]) * (stick_transition / stick_accelerator)) + (1 - stick_transition);
  }

#ifdef RX_SMOOTHING
  const float setpoint_derivative = filter_lp_pt1_step(&rx_filter, &rx_filter_state[x], (state.setpoint.axis[x] - lastsetpoint[x]) * current_kd[x] * timefactor);
#else
  const float setpoint_derivative = (state.setpoint.axis[x] - lastsetpoint[x]) * current_kd[x] * timefactor;
#endif

  const float gyro_derivative = (state.gyro.axis[x] - lastrate[x]) * current_kd[x] * timefactor * tda_compensation;
  const float dterm = (setpoint_derivative * stick_accelerator * transition_setpoint_weight) - (gyro_derivative);
  lastsetpoint[x] = state.setpoint.axis[x];
  lastrate[x] = state.gyro.axis[x];

  state.pid_d_term.axis[x] = pid_filter_dterm(x, dterm);

  state.pidoutput.axis[x] = pid_output.axis[x] = state.pid_p_term.axis[x] + state.pid_i_term.axis[x] + state.pid_d_term.axis[x];

  const float *out_limit = target.brushless ? out_limit_brushless : out_limit_brushed;
  limitf(&state.pidoutput.axis[x], out_limit[x]);
}

void pid_calc() {
  // rotates errors, originally by joelucid

  // rotation around x axis:
  ierror[1] -= ierror[2] * state.gyro.axis[0] * state.looptime;
  ierror[2] += ierror[1] * state.gyro.axis[0] * state.looptime;

  // rotation around y axis:
  ierror[2] -= ierror[0] * state.gyro.axis[1] * state.looptime;
  ierror[0] += ierror[2] * state.gyro.axis[1] * state.looptime;

  // rotation around z axis:
  ierror[0] -= ierror[1] * state.gyro.axis[2] * state.looptime;
  ierror[1] += ierror[0] * state.gyro.axis[2] * state.looptime;

  pid(0);
  pid(1);
  pid(2);
}

// below are functions used with gestures for changing pids by a percentage

// Cycle through P / I / D - The initial value is P
// The return value is the currently selected TERM (after setting the next one)
// 1: P
// 2: I
// 3: D
// The return value is used to blink the leds in main.c
int next_pid_term() {
  current_pid_term++;
  if (current_pid_term == 3) {
    current_pid_term = 0;
  }
  return current_pid_term + 1;
}

vec3_t *current_pid_term_pointer() {
  switch (current_pid_term) {
  case 0:
    return &profile_current_pid_rates()->kp;
  case 1:
    return &profile_current_pid_rates()->ki;
  case 2:
    return &profile_current_pid_rates()->kd;
  }
  return &profile_current_pid_rates()->kp;
}

// Cycle through the axis - Initial is Roll
// Return value is the selected axis, after setting the next one.
// 1: Roll
// 2: Pitch
// 3: Yaw
// The return value is used to blink the leds in main.c
int next_pid_axis() {
  const int size = 3;
  if (current_pid_axis == size - 1) {
    current_pid_axis = 0;
  } else {
#ifdef COMBINE_PITCH_ROLL_PID_TUNING
    if (current_pid_axis < 2) {
      // Skip axis == 1 which is roll, and go directly to 2 (Yaw)
      current_pid_axis = 2;
    }
#else
    current_pid_axis++;
#endif
  }

  return current_pid_axis + 1;
}

float adjust_rounded_pid(float input, float adjust_amount) {
  const float value = (int)(input * 100.0f + 0.5f);
  const float result = (float)(value + (100.0f * adjust_amount)) / 100.0f;

  if ((int)(result * 100.0f) <= 0)
    return 0;

  return result;
}

int change_pid_value(int increase) {
  float pid_adjustment = (float)PID_TUNING_ADJUST_AMOUNT;
  if (increase) {
    number_of_increments[current_pid_term][current_pid_axis]++;
  } else {
    number_of_increments[current_pid_term][current_pid_axis]--;
    pid_adjustment = -pid_adjustment;
  }

  current_pid_term_pointer()->axis[current_pid_axis] = adjust_rounded_pid(current_pid_term_pointer()->axis[current_pid_axis], pid_adjustment);

#ifdef COMBINE_PITCH_ROLL_PID_TUNING
  if (current_pid_axis == 0)
    current_pid_term_pointer()->axis[current_pid_axis + 1] = adjust_rounded_pid(current_pid_term_pointer()->axis[current_pid_axis + 1], pid_adjustment);
#endif
  return abs(number_of_increments[current_pid_term][current_pid_axis]);
}

// Increase currently selected term, for the currently selected axis, (by functions above) by 1 point
// The return value, is absolute number of times the specific term/axis was increased or decreased.  For example, if P for Roll was increased by 1 point twice,
// And then reduced by 1 point 3 times, the return value would be 1  -  The user has to track if the overall command was increase or decrease
int increase_pid() {
  return change_pid_value(1);
}

// Same as increase_pid but... you guessed it... decrease!
int decrease_pid() {
  return change_pid_value(0);
}
