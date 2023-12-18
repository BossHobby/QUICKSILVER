#include "flight/input.h"

#include <math.h>
#include <string.h>

#include "core/profile.h"
#include "flight/control.h"
#include "util/util.h"

#define pow3(x) ((x) * (x) * (x))
#define pow5(x) ((x) * (x) * (x) * (x) * (x))

#define BF_RC_RATE_INCREMENTAL 14.54f

vec3_t input_stick_vector(float rx_input[]) {
  // rotate down vector to match stick position
  const float pitch = rx_input[1] * profile.rate.level_max_angle * DEGTORAD;
  const float roll = rx_input[0] * profile.rate.level_max_angle * DEGTORAD;

  vec3_t stickvector = {
      .roll = fastsin(roll),
      .pitch = fastsin(pitch),
      .yaw = fastcos(roll) * fastcos(pitch),
  };

  const float length = (stickvector.roll * stickvector.roll + stickvector.pitch * stickvector.pitch);
  if (length > 0) {
    const float mag = 1.0f / sqrtf(length / (1 - stickvector.yaw * stickvector.yaw));
    stickvector.roll *= mag;
    stickvector.pitch *= mag;
  } else {
    stickvector.roll = 0.0f;
    stickvector.pitch = 0.0f;
  }

  // find error between stick vector and quad orientation
  // vector cross product
  return (vec3_t){
      .roll = constrain((state.GEstG.yaw * stickvector.roll) - (state.GEstG.roll * stickvector.yaw), -1.0f, 1.0f),
      .pitch = constrain(-((state.GEstG.pitch * stickvector.yaw) - (state.GEstG.yaw * stickvector.pitch)), -1.0f, 1.0f),
      .yaw = constrain(((state.GEstG.roll * stickvector.pitch) - (state.GEstG.pitch * stickvector.roll)), -1.0f, 1.0f),
  };
}

static vec3_t input_get_expo() {
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

  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE:
    acro_expo = profile_current_rates()->rate[SILVERWARE_ACRO_EXPO];
    angle_expo = profile_current_rates()->rate[SILVERWARE_ANGLE_EXPO];
    break;
  case RATE_MODE_BETAFLIGHT:
    acro_expo = profile_current_rates()->rate[BETAFLIGHT_EXPO];
    angle_expo = profile_current_rates()->rate[BETAFLIGHT_EXPO];
    break;
  case RATE_MODE_ACTUAL:
    acro_expo = profile_current_rates()->rate[ACTUAL_EXPO];
    angle_expo = profile_current_rates()->rate[ACTUAL_EXPO];
    break;
  }

  vec3_t expo;
  if (rx_aux_on(AUX_LEVELMODE)) {
    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
      expo.roll = angle_expo.roll;
      expo.pitch = acro_expo.pitch;
      expo.yaw = angle_expo.yaw;
    } else if (rx_aux_on(AUX_HORIZON)) {
      expo.roll = acro_expo.roll;
      expo.pitch = acro_expo.pitch;
      expo.yaw = angle_expo.yaw;
    } else {
      expo.roll = angle_expo.roll;
      expo.pitch = angle_expo.pitch;
      expo.yaw = angle_expo.yaw;
    }
  } else {
    expo.roll = acro_expo.roll;
    expo.pitch = acro_expo.pitch;
    expo.yaw = acro_expo.yaw;
  }
  return expo;
}

static float calc_bf_rates(const uint32_t axis, float rc, float expo) {
  const float rc_abs = fabs(rc);

  if (expo) {
    rc = rc * pow3(rc_abs) * expo + rc * (1 - expo);
  }

  float rc_rate = profile_current_rates()->rate[BETAFLIGHT_RC_RATE].axis[axis];
  if (rc_rate > 2.0f) {
    rc_rate += BF_RC_RATE_INCREMENTAL * (rc_rate - 2.0f);
  }

  float angle_rate = 200.0f * rc_rate * rc;

  const float super_rate = profile_current_rates()->rate[BETAFLIGHT_SUPER_RATE].axis[axis];
  if (super_rate) {
    const float super_factor = 1.0f / (constrain(1.0f - (rc_abs * super_rate), 0.01f, 1.00f));
    angle_rate *= super_factor;
  }

  return angle_rate * DEGTORAD;
}

static float calc_sw_rates(const uint32_t axis, float rc, float expo) {
  const float max_rate = profile_current_rates()->rate[SILVERWARE_MAX_RATE].axis[axis];
  const float rate_expo = pow3(rc) * expo + rc * (1 - expo);

  return rate_expo * max_rate * DEGTORAD;
}

static float calc_actual_rates(const uint32_t axis, float rc, float expo) {
  const float rc_abs = fabs(rc);
  const float rate_expo = rc_abs * (pow5(rc) * expo + rc * (1 - expo));

  const float center_sensitivity = profile_current_rates()->rate[ACTUAL_CENTER_SENSITIVITY].axis[axis];
  const float max_rate = profile_current_rates()->rate[ACTUAL_MAX_RATE].axis[axis];
  const float stick_movement = max(0, max_rate - center_sensitivity);

  return (rc * center_sensitivity + stick_movement * rate_expo) * DEGTORAD;
}

vec3_t input_rates_calc() {
  vec3_t rates;
  vec3_t expo = input_get_expo();

  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE:
    rates.roll = calc_sw_rates(0, state.rx_filtered.roll, expo.roll);
    rates.pitch = calc_sw_rates(1, state.rx_filtered.pitch, expo.pitch);
    rates.yaw = calc_sw_rates(2, state.rx_filtered.yaw, expo.yaw);
    break;
  case RATE_MODE_BETAFLIGHT:
    rates.roll = calc_bf_rates(0, state.rx_filtered.roll, expo.roll);
    rates.pitch = calc_bf_rates(1, state.rx_filtered.pitch, expo.pitch);
    rates.yaw = calc_bf_rates(2, state.rx_filtered.yaw, expo.yaw);
    break;
  case RATE_MODE_ACTUAL:
    rates.roll = calc_actual_rates(0, state.rx_filtered.roll, expo.roll);
    rates.pitch = calc_actual_rates(1, state.rx_filtered.pitch, expo.pitch);
    rates.yaw = calc_actual_rates(2, state.rx_filtered.yaw, expo.yaw);
    break;
  }

  return rates;
}

float input_throttle_calc(float throttle) {
  const float n = (throttle * 2.f - 1.f);
  const float expo = profile.rate.throttle_expo;
  const float mid = profile.rate.throttle_mid;
  return constrain((n * n * n * expo + n * (1.f - expo) + 1.f) * mid, 0.0f, 1.0f);
}