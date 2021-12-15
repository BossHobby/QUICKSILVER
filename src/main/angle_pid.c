#include <stdbool.h>

#include "control.h"
#include "math.h"
#include "pid.h"
#include "profile.h"
#include "util.h"

extern profile_t profile;

// code variables below
#define OUTLIMIT_FLOAT (profile.pid.small_angle.kp + profile.pid.big_angle.kp) //set angle pid output limit to sum of both P terms just in case

static float apidoutput1[ANGLE_PID_SIZE];
static float apidoutput2[ANGLE_PID_SIZE];
static float apidoutput[ANGLE_PID_SIZE];
static float lasterror[ANGLE_PID_SIZE];

float angle_pid(int x) {
  const float inverse_angle_error = 1 - fabsf(state.angleerror[x]);
  const float abs_angle_error = fabsf(state.angleerror[x]);

  // P term 1 weighted
  apidoutput1[x] = inverse_angle_error * state.angleerror[x] * profile.pid.small_angle.kp;

  // P term 2 weighted
  apidoutput2[x] = abs_angle_error * state.angleerror[x] * profile.pid.big_angle.kp;

  extern float timefactor;
  // D term 1 weighted + P term 1 weighted
  apidoutput1[x] += (state.angleerror[x] - lasterror[x]) * profile.pid.small_angle.kd * inverse_angle_error * timefactor;

  // D term 2 weighted + P term 2 weighted
  apidoutput2[x] += ((state.angleerror[x] - lasterror[x]) * profile.pid.big_angle.kd * abs_angle_error * timefactor);

  // apidoutput sum
  apidoutput[x] = apidoutput1[x] + apidoutput2[x];

  lasterror[x] = state.angleerror[x];
  limitf(&apidoutput[x], OUTLIMIT_FLOAT);

  return apidoutput[x];
}
