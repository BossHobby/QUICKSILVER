#include "motor.h"

#include "core/profile.h"
#include "driver/motor.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "project.h"
#include "util/util.h"

#ifdef BRUSHED_TARGET
#define BRUSHED_MIX_SCALING
#endif

#ifdef BRUSHLESS_TARGET
#define BRUSHLESS_MIX_SCALING
#endif

#ifdef BRUSHLESS_MIX_SCALING
#undef BRUSHED_MIX_SCALING

#ifndef AIRMODE_STRENGTH
#define AIRMODE_STRENGTH 1.0f //  Most amount of power that can be added for Airmode
#endif

#ifndef CLIPPING_LIMIT
#define CLIPPING_LIMIT 1.0f //  Most amount of power that can be pulled before clipping
#endif

#endif

#ifdef BRUSHED_MIX_SCALING
#undef BRUSHLESS_MIX_SCALING

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

#endif

extern profile_t profile;

static float motord(float in, int x) {
  float factor = profile.motor.torque_boost;
  static float lastratexx[4][4];

  float out = (+0.125f * in + 0.250f * lastratexx[x][0] - 0.250f * lastratexx[x][2] - (0.125f) * lastratexx[x][3]) * factor;

  lastratexx[x][3] = lastratexx[x][2];
  lastratexx[x][2] = lastratexx[x][1];
  lastratexx[x][1] = lastratexx[x][0];
  lastratexx[x][0] = in;

  return in + out;
}

//********************************MIXER SCALING***********************************************************
static void motor_mixer_scale_calc(float mix[4]) {

#ifdef BRUSHLESS_MIX_SCALING
  // only enable once really in the air
  if (flags.on_ground || !flags.in_air) {
    return;
  }

  float mix_min = 1000.0f;
  float mix_max = -1000.0f;

  for (int i = 0; i < 4; i++) {
    if (mix[i] < mix_min)
      mix_min = mix[i];
    if (mix[i] > mix_max)
      mix_max = mix[i];

    if (mix_min < (-AIRMODE_STRENGTH))
      mix_min = (-AIRMODE_STRENGTH);
    if (mix_max > (1 + CLIPPING_LIMIT))
      mix_max = (1 + CLIPPING_LIMIT);
  }

  float reduce_amount = 0.0f;

  const float mix_range = mix_max - mix_min;
  if (mix_range > 1.0f) {
    const float scale = 1.0f / mix_range;

    for (int i = 0; i < 4; i++)
      mix[i] *= scale;

    mix_min *= scale;
    reduce_amount = mix_min;
  } else {
    if (mix_max > 1.0f)
      reduce_amount = mix_max - 1.0f;
    else if (mix_min < 0.0f)
      reduce_amount = mix_min;
  }

  for (int i = 0; i < 4; i++)
    mix[i] -= reduce_amount;
#endif

#ifdef BRUSHED_MIX_SCALING
  // throttle reduction
  float overthrottle = 0;
  float underthrottle = 0.001f;
  static float overthrottlefilt = 0;

  for (int i = 0; i < 4; i++) {
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
    for (int i = 0; i < 4; i++) {
      mix[i] -= temp;
    }
  }

  // Brushed airmode - throttle increase
  if (flags.in_air == 1) {
    float underthrottle = 0;

    for (int i = 0; i < 4; i++) {
      if (mix[i] < underthrottle)
        underthrottle = mix[i];
    }

    // limit to half throttle max reduction
    if (underthrottle < -(float)MIX_THROTTLE_INCREASE_MAX)
      underthrottle = -(float)MIX_THROTTLE_INCREASE_MAX;

    if (underthrottle < 0.0f) {
      for (int i = 0; i < 4; i++)
        mix[i] -= underthrottle;
    }
  }
#endif
}

void motor_test_calc(bool motortest_usb, float mix[4]) {
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

void motor_mixer_calc(float mix[4]) {
  if (profile.motor.invert_yaw) {
    state.pidoutput.yaw = -state.pidoutput.yaw;
  }

#ifndef MOTOR_PLUS_CONFIGURATION
  // normal mode, we set mix according to pidoutput
  mix[MOTOR_FR] = state.throttle - state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // FR
  mix[MOTOR_FL] = state.throttle + state.pidoutput.axis[ROLL] - state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // FL
  mix[MOTOR_BR] = state.throttle - state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] - state.pidoutput.axis[YAW]; // BR
  mix[MOTOR_BL] = state.throttle + state.pidoutput.axis[ROLL] + state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // BL
#else
  // plus mode, we set mix according to pidoutput
  mix[MOTOR_FR] = state.throttle - state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // FRONT
  mix[MOTOR_FL] = state.throttle + state.pidoutput.axis[ROLL] - state.pidoutput.axis[YAW];  // LEFT
  mix[MOTOR_BR] = state.throttle - state.pidoutput.axis[ROLL] - state.pidoutput.axis[YAW];  // RIGHT
  mix[MOTOR_BL] = state.throttle + state.pidoutput.axis[PITCH] + state.pidoutput.axis[YAW]; // BACK
#endif

  for (int i = 0; i <= 3; i++) {
    if (profile.motor.torque_boost > 0.0f) {
      mix[i] = motord(mix[i], i);
    }
  }

  motor_mixer_scale_calc(mix);

  // we invert again cause it's used by the pid internally (for limit)
  if (profile.motor.invert_yaw) {
    state.pidoutput.yaw = -state.pidoutput.yaw;
  }
}

//********************************MOTOR OUTPUT***********************************************************
void motor_output_calc(float mix[4]) {
  state.thrsum = 0; // reset throttle sum for voltage monitoring logic in main loop

  // Begin for-loop to send motor commands
  for (int i = 0; i <= 3; i++) {

    mix[i] = constrainf(mix[i], 0, 1);

    // only apply digital idle if we are armed and not in motor test
    float motor_min_value = 0;
    if (!flags.on_ground && flags.arm_state && !flags.motortest_override) {
      // 0.0001 for legacy purposes, motor drivers downstream to round up
      motor_min_value = 0.0001f + (float)profile.motor.digital_idle * 0.01f;
    }

    mix[i] = mapf(mix[i], 0.0f, 1.0f, motor_min_value, profile.motor.motor_limit * 0.01f);

#ifndef NOMOTORS
    // normal mode
    motor_set(i, mix[i]);
#endif
    state.thrsum += mix[i];
  }

  // calculate throttle sum for voltage monitoring logic in main loop
  state.thrsum = state.thrsum / 4;
}
