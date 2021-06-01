#include "rx.h"

#include <math.h>

#include "control.h"
#include "drv_serial.h"
#include "filter.h"
#include "flash.h"
#include "profile.h"
#include "project.h"
#include "util.h"

extern profile_t profile;

uint8_t rx_aux_on(aux_function_t function) {
  return state.aux[profile.receiver.aux[function]];
}

float rx_expo(float in, float exp) {
  if (exp > 1)
    exp = 1;
  if (exp < -1)
    exp = -1;
  float ans = in * in * in * exp + in * (1 - exp);
  limitf(&ans, 1.0);
  return ans;
}

float rx_smoothing_hz(rx_protocol_t proto) {
#ifdef RX_UNIFIED_SERIAL
  if (proto == RX_PROTOCOL_UNIFIED_SERIAL) {
    return RX_SMOOTHING_HZ[SERIAL_PROTO_MAP[bind_storage.unified.protocol]];
  }
#endif
  return RX_SMOOTHING_HZ[proto];
}

void rx_apply_expo(void) {
  vec3_t angle_expo = {
      .roll = 0,
      .pitch = 0,
      .yaw = 0,
  };
  vec3_t acro_expo = {
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

  vec3_t expo = {
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
    state.rx.axis[0] = rx_expo(state.rx.axis[0], expo.roll);
  if (expo.pitch > 0.01)
    state.rx.axis[1] = rx_expo(state.rx.axis[1], expo.pitch);
  if (expo.yaw > 0.01)
    state.rx.axis[2] = rx_expo(state.rx.axis[2], expo.yaw);
}

void rx_precalc() {
  for (int i = 0; i < 3; ++i) {
#ifdef RX_SMOOTHING
    static float rx_temp[4] = {0, 0, 0, 0};
    lpf(&rx_temp[i], state.rx.axis[i], FILTERCALC(state.looptime, 1.0f / (float)rx_smoothing_hz(RX_PROTOCOL)));
    state.rx_filtered.axis[i] = rx_temp[i];
    limitf(&state.rx_filtered.axis[i], 1.0);
#else
    state.rx_filtered.axis[i] = state.rx.axis[i];
    limitf(&state.rx_filtered.axis[i], 1.0);
#endif

    if (profile.rate.sticks_deadband > 0.0f) {
      if (fabsf(state.rx_filtered.axis[i]) <= profile.rate.sticks_deadband) {
        state.rx_filtered.axis[i] = 0.0f;
      } else {
        if (state.rx_filtered.axis[i] >= 0) {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], profile.rate.sticks_deadband, 1, 0, 1);
        } else {
          state.rx_filtered.axis[i] = mapf(state.rx_filtered.axis[i], -profile.rate.sticks_deadband, -1, 0, -1);
        }
      }
    }
  }
  state.rx_filtered.throttle = state.rx.throttle;
}

void rx_capture_stick_range(void) {
  for (uint8_t i = 0; i < 4; i++) {
    if (state.rx.axis[i] > profile.receiver.stick_calibration_limits[i].max)
      profile.receiver.stick_calibration_limits[i].max = state.rx.axis[i]; //record max value during calibration to array
    if (state.rx.axis[i] < profile.receiver.stick_calibration_limits[i].min)
      profile.receiver.stick_calibration_limits[i].min = state.rx.axis[i]; //record min value during calibration to array
  }
}

void rx_apply_stick_calibration_scale(void) {
  for (uint8_t i = 0; i < 4; i++) {
    if (i == 3) {
      mapf(state.rx.axis[i], profile.receiver.stick_calibration_limits[i].min, profile.receiver.stick_calibration_limits[i].max, 0.f, 1.f);
    } else {
      mapf(state.rx.axis[i], profile.receiver.stick_calibration_limits[i].min, profile.receiver.stick_calibration_limits[i].max, -1.f, 1.f);
    }
  }
  constrainf(state.rx.axis[3], 0.f, 1.f); //constrain throttle min and max
}

/*stick calibration wizard sequence notes
1. user selects start stick calibration sequence

2. a timeout timer runs for 5 seconds - user is instructed to move sticks around
  somehow stick values need to be ignored
   rx_capture_stick_range() runs

3. another timeout timer runs for 5 seconds - user is instructed to input a pattern to save and apply
  if pattern is input(is this the best idea?) - save profile
4.  feedback?  failed or completed?
*/
