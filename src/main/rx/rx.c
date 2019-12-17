#include "rx.h"

#include <math.h>

#include "profile.h"
#include "project.h"
#include "util.h"

float rx_filtered[4];

extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];

extern profile_t profile;

uint8_t rx_aux_on(aux_function_t function) {
  return aux[profile.channel.aux[function]];
}

uint8_t rx_auxchange(aux_function_t function) {
  return auxchange[profile.channel.aux[function]];
}

float rcexpo(float in, float exp) {
  if (exp > 1)
    exp = 1;
  if (exp < -1)
    exp = -1;
  float ans = in * in * in * exp + in * (1 - exp);
  limitf(&ans, 1.0);
  return ans;
}

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

  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    angle_expo = profile.rate.betaflight.expo;
    acro_expo = profile.rate.betaflight.expo;
  } else {
    angle_expo = profile.rate.silverware.angle_expo;
    acro_expo = profile.rate.silverware.acro_expo;
  }

  vector_t expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  if (rx_aux_on(AUX_LEVELMODE)) {
    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
      expo.axis[0] = angle_expo.roll;
      expo.axis[1] = acro_expo.pitch;
      expo.axis[2] = angle_expo.yaw;
    } else if (rx_aux_on(AUX_HORIZON)) {
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

void calc_rx() {
  for (int i = 0; i < 3; ++i) {
#ifdef RX_SMOOTHING_HZ
    static float rx_temp[4];
    lpf(&rx_temp[i], rx[i], FILTERCALC(LOOPTIME * (float)1e-6, 1.0f / RX_SMOOTHING_HZ));
    rx_filtered[i] = rx_temp[i];
    limitf(&rx_filtered[i], 1.0);
#else
    rx_filtered[i] = rx[i];
    limitf(&rx_filtered[i], 1.0);
#endif

    if (profile.rate.sticks_deadband > 0.0f) {
      if (fabsf(rx_filtered[i]) <= profile.rate.sticks_deadband) {
        rx_filtered[i] = 0.0f;
      } else {
        if (rx_filtered[i] >= 0) {
          rx_filtered[i] = mapf(rx_filtered[i], profile.rate.sticks_deadband, 1, 0, 1);
        } else {
          rx_filtered[i] = mapf(rx_filtered[i], -profile.rate.sticks_deadband, -1, 0, -1);
        }
      }
    }
  }
  rx_filtered[3] = rx[3];
}