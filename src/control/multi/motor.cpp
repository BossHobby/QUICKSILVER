#include "control/multi/motor.h"

#include <float.h>

#include "control/control.h"
#include "control/output.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/motor.h"
#include "io/usb_configurator.h"
#include "util/util.h"

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#ifndef MIX_THROTTLE_REDUCTION_PERCENT
#define MIX_THROTTLE_REDUCTION_PERCENT 10
#endif

// limit reduction and increase to this amount ( 0.0 - 1.0)
// 0.0 = no action
// 0.5 = reduce up to 1/2 throttle
// 1.0 = reduce all the way to zero
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5
#endif

#ifndef MIX_MOTOR_MAX
#define MIX_MOTOR_MAX 1.0f
#endif

#ifndef MIX_THROTTLE_INCREASE_MAX
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif

extern profile_t profile;

static float motord(float in, int x) {
  static float lastratexx[4][4];

  const float factor = profile.motor.torque_boost;
  const float out = (+0.125f * in + 0.250f * lastratexx[x][0] - 0.250f * lastratexx[x][2] - (0.125f) * lastratexx[x][3]) * factor;
  lastratexx[x][3] = lastratexx[x][2];
  lastratexx[x][2] = lastratexx[x][1];
  lastratexx[x][1] = lastratexx[x][0];
  lastratexx[x][0] = in;

  return in + out;
}

static void motor_brushless_mixer_scale_calc(float throttle) {
  // only enable once really in the air
  if (flags.on_ground || !flags.in_air) {
    return;
  }

  float min = FLT_MAX;
  float max = FLT_MIN;

  for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
    if (state.output[i] < min) {
      min = state.output[i];
    }
    if (state.output[i] > max) {
      max = state.output[i];
    }
  }

  const float range = max - min;
  const float scale = range > 1.0f ? 1.0f / range : 1.0f;

  const float scaled_min = min * scale;
  const float scaled_max = max * scale;
  const float scaled_throttle = constrain(throttle, -scaled_min, 1.0f - scaled_max);

  for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
    state.output[i] = state.output[i] * scale + scaled_throttle;
  }
}

static void motor_brushed_mixer_scale_calc(float throttle) {
  // throttle reduction
  float overthrottle = 0;
  float underthrottle = 0.001f;
  static float overthrottlefilt = 0;

  for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
    state.output[i] += throttle;

    if (state.output[i] > overthrottle)
      overthrottle = state.output[i];
    if (state.output[i] < underthrottle)
      underthrottle = state.output[i];
  }

  overthrottle -= MIX_MOTOR_MAX;

  if (overthrottle > (float)MIX_THROTTLE_REDUCTION_MAX)
    overthrottle = (float)MIX_THROTTLE_REDUCTION_MAX;

  if (overthrottle > overthrottlefilt)
    overthrottlefilt += 0.005f;
  else
    overthrottlefilt -= 0.01f;

  if (overthrottlefilt > (float)MIX_THROTTLE_REDUCTION_MAX)
    overthrottlefilt = (float)MIX_THROTTLE_REDUCTION_MAX;
  if (overthrottlefilt < -0.1f)
    overthrottlefilt = -0.1;

  overthrottle = overthrottlefilt;

  if (overthrottle < 0.0f)
    overthrottle = -0.0001f;

  // reduce by a percentage only, so we get an inbetween performance
  overthrottle *= ((float)MIX_THROTTLE_REDUCTION_PERCENT / 100.0f);

  if (overthrottle > 0) { // exceeding max motor thrust
    float temp = overthrottle;
    for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
      state.output[i] -= temp;
    }
  }

  // Brushed airmode - throttle increase
  if (flags.in_air == 1) {
    float underthrottle = 0;

    for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
      if (state.output[i] < underthrottle)
        underthrottle = state.output[i];
    }

    // limit to half throttle max reduction
    if (underthrottle < -(float)MIX_THROTTLE_INCREASE_MAX)
      underthrottle = -(float)MIX_THROTTLE_INCREASE_MAX;

    if (underthrottle < 0.0f) {
      for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++)
        state.output[i] -= underthrottle;
    }
  }
}

void motor_test_calc(bool motortest_usb) {
  if (motortest_usb) {
    // set mix according to values we got via usb
    state.output[MOTOR_FR] = motor_test.value[MOTOR_FR];
    state.output[MOTOR_FL] = motor_test.value[MOTOR_FL];
    state.output[MOTOR_BR] = motor_test.value[MOTOR_BR];
    state.output[MOTOR_BL] = motor_test.value[MOTOR_BL];
  } else {
    // set mix according to sticks
    if (state.rx_filtered.roll < -0.5f || state.rx_filtered.pitch < -0.5f) {
      state.output[MOTOR_FR] = 0;
    } else {
      state.output[MOTOR_FR] = state.throttle;
    }

    if (state.rx_filtered.roll > 0.5f || state.rx_filtered.pitch < -0.5f) {
      state.output[MOTOR_FL] = 0;
    } else {
      state.output[MOTOR_FL] = state.throttle;
    }

    if (state.rx_filtered.roll < -0.5f || state.rx_filtered.pitch > 0.5f) {
      state.output[MOTOR_BR] = 0;
    } else {
      state.output[MOTOR_BR] = state.throttle;
    }

    if (state.rx_filtered.roll > 0.5f || state.rx_filtered.pitch > 0.5f) {
      state.output[MOTOR_BL] = 0;
    } else {
      state.output[MOTOR_BL] = state.throttle;
    }
  }
}

void motor_mixer_calc(void) {
  state.mixer_source[OUTPUT_SOURCE_THROTTLE] = state.throttle;
  state.mixer_source[OUTPUT_SOURCE_ROLL] = state.pidoutput.roll;
  state.mixer_source[OUTPUT_SOURCE_PITCH] = state.pidoutput.pitch;
  state.mixer_source[OUTPUT_SOURCE_YAW] = state.pidoutput.yaw;

  output_apply_mixer_rules();

  if (profile.motor.torque_boost > 0.0f) {
    for (uint32_t i = 0; i < MULTI_MOTOR_COUNT; i++) {
      state.output[i] = motord(state.output[i], i);
    }
  }

  if (profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT)) {
    motor_brushless_mixer_scale_calc(state.throttle);
  } else {
    motor_brushed_mixer_scale_calc(state.throttle);
  }
}
