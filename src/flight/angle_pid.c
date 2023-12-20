#include <stdbool.h>

#include "core/profile.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "math.h"
#include "util/util.h"

extern profile_t profile;

// set angle pid output limit to sum of both P terms just in case
#define OUTLIMIT_FLOAT (profile.pid.small_angle.kp + profile.pid.big_angle.kp)

float angle_pid(int x) {
  static float lasterror[ANGLE_PID_SIZE];

  const float angle_error_abs = fabsf(state.angleerror[x]);

  const float small_angle = (1 - angle_error_abs) * state.angleerror[x] * profile.pid.small_angle.kp                                          // P term weighted
                            + ((state.angleerror[x] - lasterror[x]) * profile.pid.small_angle.kd * (1 - angle_error_abs) * state.timefactor); // D term weighted

  const float big_angle = angle_error_abs * state.angleerror[x] * profile.pid.big_angle.kp                                          // P term weighted
                          + ((state.angleerror[x] - lasterror[x]) * profile.pid.big_angle.kd * angle_error_abs * state.timefactor); // D term weighted

  lasterror[x] = state.angleerror[x];

  return constrain(small_angle + big_angle, -OUTLIMIT_FLOAT, OUTLIMIT_FLOAT);
}
