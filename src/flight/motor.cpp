#include "motor.h"

#include <float.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/motor.h"
#include "flight/control.h"
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

static void motor_brushless_mixer_scale_calc(float throttle, float mix[MOTOR_PIN_MAX]) {
  // only enable once really in the air
  if (flags.on_ground || !flags.in_air) {
    return;
  }

  float min = FLT_MAX;
  float max = FLT_MIN;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (mix[i] < min) {
      min = mix[i];
    }
    if (mix[i] > max) {
      max = mix[i];
    }
  }

  const float range = max - min;
  const float scale = range > 1.0f ? 1.0f / range : 1.0f;

  const float scaled_min = min * scale;
  const float scaled_max = max * scale;
  const float scaled_throttle = constrain(throttle, -scaled_min, 1.0f - scaled_max);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    mix[i] = mix[i] * scale + scaled_throttle;
  }
}

static void motor_brushed_mixer_scale_calc(float throttle, float mix[MOTOR_PIN_MAX]) {
  // throttle reduction
  float overthrottle = 0;
  float underthrottle = 0.001f;
  static float overthrottlefilt = 0;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    mix[i] += throttle;

    if (mix[i] > overthrottle)
      overthrottle = mix[i];
    if (mix[i] < underthrottle)
      underthrottle = mix[i];
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
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      mix[i] -= temp;
    }
  }

  // Brushed airmode - throttle increase
  if (flags.in_air == 1) {
    float underthrottle = 0;

    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      if (mix[i] < underthrottle)
        underthrottle = mix[i];
    }

    // limit to half throttle max reduction
    if (underthrottle < -(float)MIX_THROTTLE_INCREASE_MAX)
      underthrottle = -(float)MIX_THROTTLE_INCREASE_MAX;

    if (underthrottle < 0.0f) {
      for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++)
        mix[i] -= underthrottle;
    }
  }
}

static void motor_mixer_scale_calc(float throttle, float mix[MOTOR_PIN_MAX]) {
  if (target.brushless) {
    return motor_brushless_mixer_scale_calc(throttle, mix);
  }
  return motor_brushed_mixer_scale_calc(throttle, mix);
}

void motor_test_calc(bool motortest_usb, float mix[MOTOR_PIN_MAX]) {
  if (motortest_usb) {
    // set mix according to values we got via usb
    mix[MOTOR_FR] = motor_test.value[MOTOR_FR];
    mix[MOTOR_FL] = motor_test.value[MOTOR_FL];
    mix[MOTOR_BR] = motor_test.value[MOTOR_BR];
    mix[MOTOR_BL] = motor_test.value[MOTOR_BL];
  } else {
    // set mix according to sticks
    if (state.rx_filtered.roll < -0.5f || state.rx_filtered.pitch < -0.5f) {
      mix[MOTOR_FR] = 0;
    } else {
      mix[MOTOR_FR] = state.throttle;
    }

    if (state.rx_filtered.roll > 0.5f || state.rx_filtered.pitch < -0.5f) {
      mix[MOTOR_FL] = 0;
    } else {
      mix[MOTOR_FL] = state.throttle;
    }

    if (state.rx_filtered.roll < -0.5f || state.rx_filtered.pitch > 0.5f) {
      mix[MOTOR_BR] = 0;
    } else {
      mix[MOTOR_BR] = state.throttle;
    }

    if (state.rx_filtered.roll > 0.5f || state.rx_filtered.pitch > 0.5f) {
      mix[MOTOR_BL] = 0;
    } else {
      mix[MOTOR_BL] = state.throttle;
    }
  }
}

void motor_mixer_calc(float mix[MOTOR_PIN_MAX]) {
  const float yaw = profile.motor.invert_yaw ? -state.pidoutput.yaw : state.pidoutput.yaw;

#ifndef MOTOR_PLUS_CONFIGURATION
  // normal mode, we set mix according to pidoutput
  mix[MOTOR_FR] = -state.pidoutput.roll - state.pidoutput.pitch + yaw; // FR
  mix[MOTOR_FL] = +state.pidoutput.roll - state.pidoutput.pitch - yaw; // FL
  mix[MOTOR_BR] = -state.pidoutput.roll + state.pidoutput.pitch - yaw; // BR
  mix[MOTOR_BL] = +state.pidoutput.roll + state.pidoutput.pitch + yaw; // BL
#else
  // plus mode, we set mix according to pidoutput
  mix[MOTOR_FR] = -state.pidoutput.pitch + yaw; // FRONT
  mix[MOTOR_FL] = +state.pidoutput.roll - yaw;  // LEFT
  mix[MOTOR_BR] = -state.pidoutput.roll - yaw;  // RIGHT
  mix[MOTOR_BL] = +state.pidoutput.pitch + yaw; // BACK
#endif

  if (profile.motor.torque_boost > 0.0f) {
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      mix[i] = motord(mix[i], i);
    }
  }

  motor_mixer_scale_calc(state.throttle, mix);
}

//********************************MOTOR OUTPUT***********************************************************
void motor_output_calc(float mix[MOTOR_PIN_MAX]) {
  state.thrsum = 0; // reset throttle sum for voltage monitoring logic in main loop

  // only apply digital idle if we are armed and not in motor test
  float motor_min_value = 0;
  if (!flags.on_ground && flags.arm_state && !flags.motortest_override) {
    // 0.0001 for legacy purposes, force motor drivers downstream to round up
    motor_min_value = 0.0001f + (float)profile.motor.digital_idle * 0.01f;
  }

  // Begin for-loop to send motor commands
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    if (!flags.motortest_override) {
      // use values as supplied in motor test mode
      mix[i] = constrain(mix[i], 0, 1);
      mix[i] = mapf(mix[i], 0.0f, 1.0f, motor_min_value, profile.motor.motor_limit * 0.01f);
    }

#ifndef NOMOTORS
    motor_set((motor_position_t)i, mix[i]);
#endif
    state.thrsum += mix[i];
  }

  // calculate throttle sum for voltage monitoring logic in main loop
  state.thrsum = state.thrsum / MOTOR_PIN_MAX;
}
