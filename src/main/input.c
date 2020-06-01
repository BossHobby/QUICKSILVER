#include "input.h"

#include <math.h>
#include <string.h>

#include "control.h"
#include "profile.h"
#include "util.h"

extern float Q_rsqrt(float number);
extern profile_t profile;

// error vector between stick position and quad orientation
// this is the output of this function
float errorvect[3];

// cache the last result so it does not get calculated everytime
static float last_rx[2] = {13.13f, 12.12f};
static float stickvector[3] = {0, 0, 1};

void input_stick_vector(float rx_input[], float maxangle) {
  // only compute stick rotation if values changed
  if (last_rx[0] != rx_input[0] || last_rx[1] != rx_input[1]) {
    last_rx[0] = rx_input[0];
    last_rx[1] = rx_input[1];

    float pitch, roll;

    // rotate down vector to match stick position
    pitch = rx_input[1] * profile.rate.level_max_angle * DEGTORAD + (float)TRIM_PITCH * DEGTORAD;
    roll = rx_input[0] * profile.rate.level_max_angle * DEGTORAD + (float)TRIM_ROLL * DEGTORAD;

    stickvector[0] = fastsin(roll);
    stickvector[1] = fastsin(pitch);
    stickvector[2] = fastcos(roll) * fastcos(pitch);

    float mag2 = (stickvector[0] * stickvector[0] + stickvector[1] * stickvector[1]);

    if (mag2 > 0.001f) {
      mag2 = Q_rsqrt(mag2 / (1 - stickvector[2] * stickvector[2]));
    } else
      mag2 = 0.707f;

    stickvector[0] *= mag2;
    stickvector[1] *= mag2;

#ifdef INVERTED_ENABLE
    extern int pwmdir;

    if (pwmdir == REVERSE) {
      stickvector[0] = -stickvector[0];
      stickvector[1] = -stickvector[1];
      stickvector[2] = -stickvector[2];
    }
#endif
  }

  // find error between stick vector and quad orientation
  // vector cross product
  errorvect[1] = -((state.GEstG.axis[1] * stickvector[2]) - (state.GEstG.axis[2] * stickvector[1]));
  errorvect[0] = (state.GEstG.axis[2] * stickvector[0]) - (state.GEstG.axis[0] * stickvector[2]);

  // some limits just in case

  limitf(&errorvect[0], 1.0);
  limitf(&errorvect[1], 1.0);

  // fix to recover if triggered inverted
  // the vector cross product results in zero for opposite vectors, so it's bad at 180 error
  // without this the quad will not invert if angle difference = 180

#ifdef INVERTED_ENABLE

  static int flip_active_once = 0;
  static int flipaxis = 0;
  static int flipdir = 0;
  int flip_active = 0;

#define rollrate 2.0f
#define g_treshold 0.125f
#define roll_bias 0.25f

  if (rx_aux_on(AUX_FN_INVERTED) && (state.GEstG.axis[2] > g_treshold)) {
    flip_active = 1;
    // rotate around axis with larger leaning angle

    if (flipdir) {
      errorvect[flipaxis] = rollrate;
    } else {
      errorvect[flipaxis] = -rollrate;
    }

  } else if (!rx_aux_on(AUX_FN_INVERTED) && (state.GEstG.axis[2] < -g_treshold)) {
    flip_active = 1;

    if (flipdir) {
      errorvect[flipaxis] = -rollrate;
    } else {
      errorvect[flipaxis] = rollrate;
    }

  } else
    flip_active_once = 0;

  // set common things here to avoid duplication
  if (flip_active) {
    if (!flip_active_once) {
      // check which axis is further from center, with a bias towards roll
      // because a roll flip does not leave the quad facing the wrong way
      if (fabsf(state.GEstG.axis[0]) + roll_bias > fabsf(state.GEstG.axis[1])) {
        // flip in roll axis
        flipaxis = 0;
      } else
        flipaxis = 1;

      if (state.GEstG.axis[flipaxis] > 0)
        flipdir = 1;
      else
        flipdir = 0;

      flip_active_once = 1;
    }

    // set the error in other axis to return to zero
    errorvect[!flipaxis] = state.GEstG.axis[!flipaxis];
  }
#endif
}

static float calc_bf_rates(int axis) {
#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

  float rcRate, superExpo;
  if (axis == ROLL) {
    rcRate = profile.rate.betaflight.rc_rate.roll;
    superExpo = profile.rate.betaflight.super_rate.roll;
  } else if (axis == PITCH) {
    rcRate = profile.rate.betaflight.rc_rate.pitch;
    superExpo = profile.rate.betaflight.super_rate.pitch;
  } else {
    rcRate = profile.rate.betaflight.rc_rate.yaw;
    superExpo = profile.rate.betaflight.super_rate.yaw;
  }
  if (rcRate > 2.0f) {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }
  const float rcCommandfAbs = state.rx_filtered.axis[axis] > 0 ? state.rx_filtered.axis[axis] : -state.rx_filtered.axis[axis];
  float angleRate = 200.0f * rcRate * state.rx_filtered.axis[axis];
  if (superExpo) {
    const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superExpo), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }
  return constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float)DEGTORAD;
}

void input_rates_calc(float rates[]) {
  // high-low rates switch
  float rate_multiplier = 1.0;
  if (rx_aux_on(AUX_HIGH_RATES) <= 0) {
    rate_multiplier = profile.rate.low_rate_mulitplier;
  }

  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    rates[0] = rate_multiplier * calc_bf_rates(0);
    rates[1] = rate_multiplier * calc_bf_rates(1);
    rates[2] = rate_multiplier * calc_bf_rates(2);
  } else {
    rates[0] = rate_multiplier * state.rx_filtered.axis[0] * profile.rate.silverware.max_rate.roll * DEGTORAD;
    rates[1] = rate_multiplier * state.rx_filtered.axis[1] * profile.rate.silverware.max_rate.pitch * DEGTORAD;
    rates[2] = rate_multiplier * state.rx_filtered.axis[2] * profile.rate.silverware.max_rate.yaw * DEGTORAD;
  }
}