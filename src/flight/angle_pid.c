#include <stdbool.h>

#include "core/profile.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "math.h"
#include "util/util.h"

extern profile_t profile;

// set angle pid output limit to sum of both P terms just in case
#define OUTLIMIT_FLOAT (profile.pid.small_angle.kp + profile.pid.big_angle.kp)

static float apidoutput1[ANGLE_PID_SIZE];
static float apidoutput2[ANGLE_PID_SIZE];
static float apidoutput[ANGLE_PID_SIZE];
static float lasterror[ANGLE_PID_SIZE];

extern float timefactor;

float angle_pid(int x) {
  const float angle_error_abs = fabsf(state.angleerror[x]);

  // P term 1 weighted
  apidoutput1[x] = (1 - angle_error_abs) * state.angleerror[x] * profile.pid.small_angle.kp;

  // P term 2 weighted
  apidoutput2[x] = angle_error_abs * state.angleerror[x] * profile.pid.big_angle.kp;

  // D term 1 weighted + P term 1 weighted
  apidoutput1[x] += (state.angleerror[x] - lasterror[x]) * profile.pid.small_angle.kd * (1 - angle_error_abs) * timefactor;

  // D term 2 weighted + P term 2 weighted
  apidoutput2[x] += ((state.angleerror[x] - lasterror[x]) * profile.pid.big_angle.kd * angle_error_abs * timefactor);

  // apidoutput sum
  apidoutput[x] = apidoutput1[x] + apidoutput2[x];

  lasterror[x] = state.angleerror[x];
  limitf(&apidoutput[x], OUTLIMIT_FLOAT);

  return apidoutput[x];
}
