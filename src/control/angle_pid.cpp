#include "control/angle_pid.h"

#include <math.h>

#include "control/control.h"
#include "core/profile.h"
#include "util/util.h"

extern profile_t profile;

// Set angle PID output limit to the sum of both P terms just in case.
#define OUTLIMIT_FLOAT (profile.pid.small_angle.kp + profile.pid.big_angle.kp)
#define ANGLE_PID_DTIME_FACTOR 0.0032f

float angle_pid(int x) {
  static vec3_t lasterror;

  const float angle_error_abs = fabsf(state.angle_error.axis[x]);

  const float timefactor = ANGLE_PID_DTIME_FACTOR * state.looptime_inverse;
  // Navigation targets arrive slower than the control loop. Differentiate
  // measured rotation during RTH so target steps cannot kick the D term.
  const float derivative = state.rth_active
      ? -state.gyro.axis[x] * ANGLE_PID_DTIME_FACTOR
      : (state.angle_error.axis[x] - lasterror.axis[x]) * timefactor;

  const float small_angle = (1 - angle_error_abs) * state.angle_error.axis[x] * profile.pid.small_angle.kp
                            + derivative * profile.pid.small_angle.kd * (1 - angle_error_abs);

  const float big_angle = angle_error_abs * state.angle_error.axis[x] * profile.pid.big_angle.kp
                          + derivative * profile.pid.big_angle.kd * angle_error_abs;

  lasterror.axis[x] = state.angle_error.axis[x];

  return constrain(small_angle + big_angle, -OUTLIMIT_FLOAT, OUTLIMIT_FLOAT);
}
