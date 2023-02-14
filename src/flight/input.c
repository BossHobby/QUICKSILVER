#include "flight/input.h"

#include <math.h>
#include <string.h>

#include "core/profile.h"
#include "flight/control.h"
#include "util/util.h"

#define BF_SETPOINT_RATE_LIMIT 1998.0f
#define BF_RC_RATE_INCREMENTAL 14.54f

extern profile_t profile;

// cache the last result so it does not get calculated everytime
static float last_rx[2] = {13.13f, 12.12f};
static float stickvector[3] = {0, 0, 1};

void input_stick_vector(float rx_input[], float maxangle) {
  // only compute stick rotation if values changed
  if (last_rx[0] != rx_input[0] || last_rx[1] != rx_input[1]) {
    last_rx[0] = rx_input[0];
    last_rx[1] = rx_input[1];

    float pitch, roll;

    // rotate down vector to match stick position
    pitch = rx_input[1] * profile.rate.level_max_angle * DEGTORAD;
    roll = rx_input[0] * profile.rate.level_max_angle * DEGTORAD;

    stickvector[0] = fastsin(roll);
    stickvector[1] = fastsin(pitch);
    stickvector[2] = fastcos(roll) * fastcos(pitch);

    float mag2 = (stickvector[0] * stickvector[0] + stickvector[1] * stickvector[1]);

    if (mag2 > 0.001f) {
      mag2 = Q_rsqrt(mag2 / (1 - stickvector[2] * stickvector[2]));
    } else
      mag2 = 0.707f;

    stickvector[0] *= mag2;
    stickvector[1] *= mag2;
  }

  // find error between stick vector and quad orientation
  // vector cross product
  state.errorvect.axis[1] = -((state.GEstG.axis[1] * stickvector[2]) - (state.GEstG.axis[2] * stickvector[1]));
  state.errorvect.axis[0] = (state.GEstG.axis[2] * stickvector[0]) - (state.GEstG.axis[0] * stickvector[2]);

  // some limits just in case

  limitf(&state.errorvect.axis[0], 1.0);
  limitf(&state.errorvect.axis[1], 1.0);

  // fix to recover if triggered inverted
  // the vector cross product results in zero for opposite vectors, so it's bad at 180 error
  // without this the quad will not invert if angle difference = 180
}

float input_apply_expo(float in, float expo) {
  if (expo < 0.01f) {
    return in;
  }

  limitf(&expo, 1.0);

  float result = 0.0f;

  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE:
    result = in * in * in * expo + in * (1 - expo);
    break;
  case RATE_MODE_BETAFLIGHT:
    result = fabsf(in) * in * in * in * expo + in * (1 - expo);
    break;
  case RATE_MODE_ACTUAL:
    result = fabsf(in) * (powf(in, 5) * expo + in * (1 - expo));
    break;
  }

  limitf(&result, 1.0);
  return result;
}

static void input_get_expo(vec3_t *expo) {
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

  if (rx_aux_on(AUX_LEVELMODE)) {
    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) {
      expo->roll = angle_expo.roll;
      expo->pitch = acro_expo.pitch;
      expo->yaw = angle_expo.yaw;
    } else if (rx_aux_on(AUX_HORIZON)) {
      expo->roll = acro_expo.roll;
      expo->pitch = acro_expo.pitch;
      expo->yaw = angle_expo.yaw;
    } else {
      expo->roll = angle_expo.roll;
      expo->pitch = angle_expo.pitch;
      expo->yaw = angle_expo.yaw;
    }
  } else {
    expo->roll = acro_expo.roll;
    expo->pitch = acro_expo.pitch;
    expo->yaw = acro_expo.yaw;
  }
}

static void calc_bf_rates(vec3_t *rates) {
  vec3_t expo;
  input_get_expo(&expo);

  for (uint32_t i = 0; i < 3; i++) {
    const float rate = input_apply_expo(state.rx_filtered.axis[i], expo.axis[i]);

    float rc_rate = profile_current_rates()->rate[BETAFLIGHT_RC_RATE].axis[i];
    if (rc_rate > 2.0f) {
      rc_rate += BF_RC_RATE_INCREMENTAL * (rc_rate - 2.0f);
    }

    float angleRate = 200.0f * rc_rate * rate;

    const float super_rate = profile_current_rates()->rate[BETAFLIGHT_SUPER_RATE].axis[i];
    if (super_rate) {
      const float super_factor = 1.0f / (constrainf(1.0f - (fabsf(rate) * super_rate), 0.01f, 1.00f));
      angleRate *= super_factor;
    }

    rates->axis[i] = constrainf(angleRate, -BF_SETPOINT_RATE_LIMIT, BF_SETPOINT_RATE_LIMIT) * DEGTORAD;
  }
}

static void calc_sw_rates(vec3_t *rates) {
  vec3_t expo;
  input_get_expo(&expo);

  rates->roll = input_apply_expo(state.rx_filtered.roll, expo.roll) * profile_current_rates()->rate[SILVERWARE_MAX_RATE].roll * DEGTORAD;
  rates->pitch = input_apply_expo(state.rx_filtered.pitch, expo.pitch) * profile_current_rates()->rate[SILVERWARE_MAX_RATE].pitch * DEGTORAD;
  rates->yaw = input_apply_expo(state.rx_filtered.yaw, expo.yaw) * profile_current_rates()->rate[SILVERWARE_MAX_RATE].yaw * DEGTORAD;
}

static void calc_actual_rates(vec3_t *rates) {
  vec3_t expo;
  input_get_expo(&expo);

  for (uint32_t i = 0; i < 3; i++) {
    const float rate_no_expo = state.rx_filtered.axis[i];
    const float rate_expo = input_apply_expo(rate_no_expo, expo.axis[i]);

    const float center_sensitivity = profile_current_rates()->rate[ACTUAL_CENTER_SENSITIVITY].axis[i];
    const float max_rate = profile_current_rates()->rate[ACTUAL_MAX_RATE].axis[i];

    float stick_movement = max_rate - center_sensitivity;
    if (stick_movement < 0) {
      stick_movement = 0;
    }

    rates->axis[i] = (rate_no_expo * center_sensitivity + stick_movement * rate_expo) * DEGTORAD;
  }
}

void input_rates_calc(vec3_t *rates) {
  switch (profile_current_rates()->mode) {
  case RATE_MODE_SILVERWARE:
    calc_sw_rates(rates);
    break;
  case RATE_MODE_BETAFLIGHT:
    calc_bf_rates(rates);
    break;
  case RATE_MODE_ACTUAL:
    calc_actual_rates(rates);
    break;
  }
}

float input_throttle_calc(float throttle) {
  const float n = (throttle * 2.f - 1.f);
  const float expo = profile.rate.throttle_expo;
  const float mid = profile.rate.throttle_mid;
  return constrainf((n * n * n * expo + n * (1.f - expo) + 1.f) * mid, 0.0f, 1.0f);
}