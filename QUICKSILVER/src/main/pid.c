/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <stdbool.h>
#include <stdlib.h>

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "defines.h"
#include "led.h"
#include "pid.h"
#include "profile.h"
#include "util.h"

//************************************Setpoint Weight****************************************
#ifdef BRUSHLESS_TARGET

/// output limit
const float outlimit[PIDNUMBER] = {0.8, 0.8, 0.4};

// limit of integral term (abs)
const float integrallimit[PIDNUMBER] = {0.8, 0.8, 0.4};

#else //BRUSHED TARGET

// "p term setpoint weighting" 0.0 - 1.0 where 1.0 = normal pid
#define ENABLE_SETPOINT_WEIGHTING
//            Roll   Pitch   Yaw
//float b[3] = { 0.97 , 0.98 , 0.95};   //BRUSHED RACE
float b[3] = {0.93, 0.93, 0.9}; //BRUSHED FREESTYLE

/// output limit
const float outlimit[PIDNUMBER] = {1.7, 1.7, 0.5};

// limit of integral term (abs)
const float integrallimit[PIDNUMBER] = {1.7, 1.7, 0.5};

#endif

//#define RECTANGULAR_RULE_INTEGRAL
//#define MIDPOINT_RULE_INTEGRAL
#define SIMPSON_RULE_INTEGRAL

//#define ANTI_WINDUP_DISABLE

// non changable things below
extern profile_t profile;

int number_of_increments[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
int current_pid_axis = 0;
int current_pid_term = 0;

float ierror[PIDNUMBER] = {0, 0, 0};
float pidoutput[PIDNUMBER];
float setpoint[PIDNUMBER];
static float lasterror[PIDNUMBER];
float v_compensation = 1.00;

extern float error[PIDNUMBER];
extern float setpoint[PIDNUMBER];
extern float looptime;
extern float gyro[3];
extern int onground;
extern float looptime;
extern int in_air;
extern float vbattfilt;
extern float vbattfilt_corr;
extern float vbatt_comp;

// multiplier for pids at 3V - for PID_VOLTAGE_COMPENSATION - default 1.33f from H101 code
#define PID_VC_FACTOR 1.33f

#ifdef SIMPSON_RULE_INTEGRAL
static float lasterror2[PIDNUMBER];
#endif

float timefactor;

// pid calculation for acro ( rate ) mode
// input: error[x] = setpoint - gyro
// output: pidoutput[x] = change required from motors
float pid(int x) {
  extern int flipstage;
  if ((rx_aux_on(AUX_LEVELMODE)) && (!rx_aux_on(AUX_RACEMODE))) { //in level mode or horizon but not racemode
    if ((onground) || (in_air == 0) || (flipstage > 0)) {         // and while on the ground or while turtle is operating...
      ierror[x] *= 0.98f;
    } // wind down the integral error
  } else {
    if (onground || (flipstage > 0))
      ierror[x] *= 0.98f; //in acro mode - only wind down integral when idle up is off and throttle is 0
  }

  int iwindup = 0; // (iwidup = 0  windup is permitted)   (iwindup = 1 windup is squashed)
  if ((pidoutput[x] >= outlimit[x]) && (error[x] > 0)) {
    iwindup = 1;
  }

  if ((pidoutput[x] == -outlimit[x]) && (error[x] < 0)) {
    iwindup = 1;
  }

#ifdef I_TERM_RELAX //  Roll - Pitch  Setpoint based I term relax method
#ifndef RELAX_FACTOR
#define RELAX_FACTOR 10 //  5.7 degrees/s
#endif
#ifndef RELAX_FREQUENCY
#define RELAX_FREQUENCY_HZ 20
#endif
  static float avgSetpoint[2];
  if (x < 2) {
    lpf(&avgSetpoint[x], setpoint[x], FILTERCALC((LOOPTIME * 1e-6f), 1.0f / (float)RELAX_FREQUENCY_HZ)); // 20 Hz filter
    const float hpfSetpoint = setpoint[x] - avgSetpoint[x];
    if (fabsf(hpfSetpoint) > (float)RELAX_FACTOR * 1e-2f) {
      iwindup = 1;
    }
  }
#endif

  if (!iwindup) {
#ifdef MIDPOINT_RULE_INTEGRAL
    // trapezoidal rule instead of rectangular
    ierror[x] = ierror[x] + (error[x] + lasterror[x]) * 0.5f * profile_current_pid_rates()->ki.axis[x] * looptime;
    lasterror[x] = error[x];
#endif

#ifdef RECTANGULAR_RULE_INTEGRAL
    ierror[x] = ierror[x] + error[x] * profile_current_pid_rates()->ki.axis[x] * looptime;
    lasterror[x] = error[x];
#endif

#ifdef SIMPSON_RULE_INTEGRAL
    // assuming similar time intervals
    ierror[x] = ierror[x] + 0.166666f * (lasterror2[x] + 4 * lasterror[x] + error[x]) * profile_current_pid_rates()->ki.axis[x] * looptime;
    lasterror2[x] = lasterror[x];
    lasterror[x] = error[x];
#endif
  }

  limitf(&ierror[x], integrallimit[x]);

#ifdef ENABLE_SETPOINT_WEIGHTING
  // P term
  pidoutput[x] = error[x] * (b[x]) * profile_current_pid_rates()->kp.axis[x];
  // b
  pidoutput[x] += -(1.0f - b[x]) * profile_current_pid_rates()->kp.axis[x] * gyro[x];
#else
  // P term with b disabled
  pidoutput[x] = error[x] * profile_current_pid_rates()->kp.axis[x];
#endif

  // I term
  pidoutput[x] += ierror[x];

  // D term
  // skip yaw D term if not set
  if (profile_current_pid_rates()->kd.axis[x] > 0) {

#if (defined DTERM_LPF_1ST_HZ && !defined ADVANCED_PID_CONTROLLER)
    float dterm;
    static float lastrate[3];
    static float dlpf[3] = {0};

    dterm = -(gyro[x] - lastrate[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor;
    lastrate[x] = gyro[x];
    lpf(&dlpf[x], dterm, FILTERCALC(looptime, 1.0f / DTERM_LPF_1ST_HZ));
    pidoutput[x] += dlpf[x];
#endif

#if (defined DTERM_LPF_1ST_HZ && defined ADVANCED_PID_CONTROLLER)
    extern float rxcopy[4];
    float dterm;
    float transitionSetpointWeight[3];
    float stickAccelerator[3];
    float stickTransition[3];
    if (rx_aux_on(AUX_STICK_BOOST_PROFILE)) {
      stickAccelerator[x] = profile.pid.stick_rates[STICK_PROFILE_ON].accelerator.axis[x];
      stickTransition[x] = profile.pid.stick_rates[STICK_PROFILE_ON].transition.axis[x];
    } else {
      stickAccelerator[x] = profile.pid.stick_rates[STICK_PROFILE_OFF].accelerator.axis[x];
      stickTransition[x] = profile.pid.stick_rates[STICK_PROFILE_OFF].transition.axis[x];
    }
    if (stickAccelerator[x] < 1) {
      transitionSetpointWeight[x] = (fabsf(rxcopy[x]) * stickTransition[x]) + (1 - stickTransition[x]);
    } else {
      transitionSetpointWeight[x] = (fabsf(rxcopy[x]) * (stickTransition[x] / stickAccelerator[x])) + (1 - stickTransition[x]);
    }
    static float lastrate[3];
    static float lastsetpoint[3];
    static float dlpf[3] = {0};
    static float setpoint_derivative[3];

    setpoint_derivative[x] = (setpoint[x] - lastsetpoint[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor;
#ifdef RX_SMOOTHING_HZ
    lpf(&setpoint_derivative[x], setpoint_derivative[x], FILTERCALC(LOOPTIME * (float)1e-6, 1.0f / RX_SMOOTHING_HZ));
#endif
    dterm = (setpoint_derivative[x] * stickAccelerator[x] * transitionSetpointWeight[x]) - ((gyro[x] - lastrate[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor);
    lastsetpoint[x] = setpoint[x];
    lastrate[x] = gyro[x];
    lpf(&dlpf[x], dterm, FILTERCALC(looptime, 1.0f / DTERM_LPF_1ST_HZ));
    pidoutput[x] += dlpf[x];
#endif

#if (defined DTERM_LPF_2ND_HZ && !defined ADVANCED_PID_CONTROLLER)
    float dterm;
    static float lastrate[3];
    float lpf2(float in, int num);

    dterm = -(gyro[x] - lastrate[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor;
    lastrate[x] = gyro[x];
    dterm = lpf2(dterm, x);
    pidoutput[x] += dterm;
#endif

#if (defined DTERM_LPF_2ND_HZ && defined ADVANCED_PID_CONTROLLER)
    extern float rxcopy[4];
    float dterm;
    float transitionSetpointWeight[3];
    float stickAccelerator[3];
    float stickTransition[3];
    if (rx_aux_on(AUX_STICK_BOOST_PROFILE)) {
      stickAccelerator[x] = profile.pid.stick_rates[STICK_PROFILE_ON].accelerator.axis[x];
      stickTransition[x] = profile.pid.stick_rates[STICK_PROFILE_ON].transition.axis[x];
    } else {
      stickAccelerator[x] = profile.pid.stick_rates[STICK_PROFILE_OFF].accelerator.axis[x];
      stickTransition[x] = profile.pid.stick_rates[STICK_PROFILE_OFF].transition.axis[x];
    }
    if (stickAccelerator[x] < 1) {
      transitionSetpointWeight[x] = (fabsf(rxcopy[x]) * stickTransition[x]) + (1 - stickTransition[x]);
    } else {
      transitionSetpointWeight[x] = (fabsf(rxcopy[x]) * (stickTransition[x] / stickAccelerator[x])) + (1 - stickTransition[x]);
    }
    static float lastrate[3];
    static float lastsetpoint[3];
    float lpf2(float in, int num);
    static float setpoint_derivative[3];

    setpoint_derivative[x] = (setpoint[x] - lastsetpoint[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor;
#ifdef RX_SMOOTHING_HZ
    lpf(&setpoint_derivative[x], setpoint_derivative[x], FILTERCALC(LOOPTIME * (float)1e-6, 1.0f / RX_SMOOTHING_HZ));
#endif
    dterm = (setpoint_derivative[x] * stickAccelerator[x] * transitionSetpointWeight[x]) - ((gyro[x] - lastrate[x]) * profile_current_pid_rates()->kd.axis[x] * timefactor);
    lastsetpoint[x] = setpoint[x];
    lastrate[x] = gyro[x];
    dterm = lpf2(dterm, x);
    pidoutput[x] += dterm;
#endif
  }

  if (profile.voltage.pid_voltage_compensation)
    pidoutput[x] *= v_compensation;

  limitf(&pidoutput[x], outlimit[x]);

  return pidoutput[x];
}

// calculate change from ideal loop time
// 0.0032f is there for legacy purposes, should be 0.001f = looptime
// this is called in advance as an optimization because it has division
void pid_precalc() {
  timefactor = 0.0032f / looptime;

  if (profile.voltage.pid_voltage_compensation) {

#ifdef EXACT_VOLTS
    //v_compensation = mapf((vbattfilt / (float)lipo_cell_count), 2.5, 3.85, PID_VC_FACTOR, 1.00);
    extern float throttle;
    v_compensation = mapf(throttle, 0.4, 1.0, 1.0, .80);
    if (v_compensation > 1.00f)
      v_compensation = 1.00;
    if (v_compensation < 0.80f)
      v_compensation = 0.80;
#endif
#ifdef FILTERED_VOLTS
    extern float lipo_cell_count;
    v_compensation = mapf((vbattfilt_corr / (float)lipo_cell_count), 2.5, 3.85, PID_VC_FACTOR, 1.00);
    if (v_compensation > PID_VC_FACTOR)
      v_compensation = PID_VC_FACTOR;
    if (v_compensation < 1.00f)
      v_compensation = 1.00;
#endif
#ifdef FUELGAUGE_VOLTS
    extern float lipo_cell_count;
    v_compensation = mapf((vbatt_comp / (float)lipo_cell_count), 2.5, 3.85, PID_VC_FACTOR, 1.00);
    if (v_compensation > PID_VC_FACTOR)
      v_compensation = PID_VC_FACTOR;
    if (v_compensation < 1.00f)
      v_compensation = 1.00;
#endif

#ifdef LEVELMODE_PID_ATTENUATION
    if (rx_aux_on(AUX_LEVELMODE))
      v_compensation *= LEVELMODE_PID_ATTENUATION;
#endif
  }
}

#ifndef DTERM_LPF_2ND_HZ
#define DTERM_LPF_2ND_HZ 99
#endif

//the compiler calculates these
static float two_one_minus_alpha = 2 * FILTERCALC((LOOPTIME * 1e-6), (1.0f / DTERM_LPF_2ND_HZ));
static float one_minus_alpha_sqr = (FILTERCALC((LOOPTIME * 1e-6), (1.0f / DTERM_LPF_2ND_HZ))) * (FILTERCALC((LOOPTIME * 1e-6), (1.0f / DTERM_LPF_2ND_HZ)));
static float alpha_sqr = (1 - FILTERCALC((LOOPTIME * 1e-6), (1.0f / DTERM_LPF_2ND_HZ))) * (1 - FILTERCALC((LOOPTIME * 1e-6), (1.0f / DTERM_LPF_2ND_HZ)));

static float last_out[3], last_out2[3];

float lpf2(float in, int num) {

  float ans = in * alpha_sqr + two_one_minus_alpha * last_out[num] - one_minus_alpha_sqr * last_out2[num];

  last_out2[num] = last_out[num];
  last_out[num] = ans;

  return ans;
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

vector_t *current_pid_term_pointer() {
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

float adjust_rounded_pid(float input, float adjust_amount){
    float result;
    float value = (int)(input * 100.0f + 0.5f);
    result = (float)(value+(100.0f * adjust_amount)) / 100.0f;
    if ((int)(result*100.0f) <= 0) return 0;
    else return result;
}

int change_pid_value(int increase) {
  float pid_adjustment = (float)PID_TUNING_ADJUST_AMOUNT;
  if (increase) {
    number_of_increments[current_pid_term][current_pid_axis]++;
  } else {
    number_of_increments[current_pid_term][current_pid_axis]--;
    pid_adjustment = -pid_adjustment;
  }

  current_pid_term_pointer()->axis[current_pid_axis] = adjust_rounded_pid(current_pid_term_pointer()->axis[current_pid_axis] , pid_adjustment);

#ifdef COMBINE_PITCH_ROLL_PID_TUNING
  if (current_pid_axis == 0)
    current_pid_term_pointer()->axis[current_pid_axis + 1] = adjust_rounded_pid(current_pid_term_pointer()->axis[current_pid_axis + 1] , pid_adjustment);
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

void rotateErrors() {
#ifdef YAW_FIX
  // rotation around x axis:
  ierror[1] -= ierror[2] * gyro[0] * looptime;
  ierror[2] += ierror[1] * gyro[0] * looptime;

  // rotation around y axis:
  ierror[2] -= ierror[0] * gyro[1] * looptime;
  ierror[0] += ierror[2] * gyro[1] * looptime;

  // rotation around z axis:
  ierror[0] -= ierror[1] * gyro[2] * looptime;
  ierror[1] += ierror[0] * gyro[2] * looptime;
#endif
}
