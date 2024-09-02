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
  static vec3_t lasterror;

  const float angle_error_abs = fabsf(state.angle_error.axis[x]);

  const float small_angle = (1 - angle_error_abs) * state.angle_error.axis[x] * profile.pid.small_angle.kp                                               // P term weighted
                            + ((state.angle_error.axis[x] - lasterror.axis[x]) * profile.pid.small_angle.kd * (1 - angle_error_abs) * state.timefactor); // D term weighted

  const float big_angle = angle_error_abs * state.angle_error.axis[x] * profile.pid.big_angle.kp                                               // P term weighted
                          + ((state.angle_error.axis[x] - lasterror.axis[x]) * profile.pid.big_angle.kd * angle_error_abs * state.timefactor); // D term weighted

  lasterror.axis[x] = state.angle_error.axis[x];

  return constrain(small_angle + big_angle, -OUTLIMIT_FLOAT, OUTLIMIT_FLOAT);
}
