#include <stdbool.h>

#include "math.h"
#include "pid.h"
#include "profile.h"
#include "util.h"

#define APIDNUMBER 2

extern profile_t profile;

// code variables below
#define OUTLIMIT_FLOAT (profile.pid.small_angle.kp + profile.pid.big_angle.kp) //set angle pid output limit to sum of both P terms just in case

float apidoutput1[APIDNUMBER];
float apidoutput2[APIDNUMBER];
float angleerror[APIDNUMBER];
float lasterror[APIDNUMBER];
float apidoutput[APIDNUMBER];

float apid(int x) {
  // P term 1 weighted
  apidoutput1[x] = (1 - fabsf(angleerror[x])) * angleerror[x] * profile.pid.small_angle.kp;

  // P term 2 weighted
  apidoutput2[x] = fabsf(angleerror[x]) * angleerror[x] * profile.pid.big_angle.kp;

  extern float timefactor;
  // D term 1 weighted + P term 1 weighted
  apidoutput1[x] += (angleerror[x] - lasterror[x]) * profile.pid.small_angle.kd * (1 - fabsf(angleerror[x])) * timefactor;

  // D term 2 weighted + P term 2 weighted
  apidoutput2[x] += ((angleerror[x] - lasterror[x]) * profile.pid.big_angle.kd * fabsf(angleerror[x]) * timefactor);

  // apidoutput sum
  apidoutput[x] = apidoutput1[x] + apidoutput2[x];

  lasterror[x] = angleerror[x];
  limitf(&apidoutput[x], OUTLIMIT_FLOAT);

  return apidoutput[x];
}
