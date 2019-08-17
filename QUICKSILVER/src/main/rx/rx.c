#include "rx.h"

#include "profile.h"
#include "project.h"
#include "util.h"

extern float rx[4];
extern char aux[AUXNUMBER];
extern profile_t profile;

void rx_apply_expo(void) {
  vector_t angle_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  vector_t acro_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };

  if (profile.rate_mode == RATE_MODE_BETAFLIGHT) {
    angle_expo = profile.betaflight_rate.expo;
    acro_expo = profile.betaflight_rate.expo;
  } else {
    angle_expo = profile.silverware_rate.angle_expo;
    acro_expo = profile.silverware_rate.acro_expo;
  }

  vector_t expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  if (aux[LEVELMODE]) {
    if (aux[RACEMODE] && !aux[HORIZON]) {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = acro_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    } else if (aux[HORIZON]) {
      expo.axis[0] = acro_expo.roll;
      expo.axis[1] = acro_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    } else {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = angle_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    }
  } else {
    expo.axis[0] = acro_expo.roll;
    expo.axis[1] = acro_expo.pitch;
    expo.axis[2] = acro_expo.yaw;
  }

  if (expo.roll > 0.01)
    rx[0] = rcexpo(rx[0], expo.roll);
  if (expo.pitch > 0.01)
    rx[1] = rcexpo(rx[1], expo.pitch);
  if (expo.yaw > 0.01)
    rx[2] = rcexpo(rx[2], expo.yaw);
}