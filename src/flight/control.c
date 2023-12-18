#include "flight/control.h"

#include <math.h>
#include <stdint.h>

#include "angle_pid.h"
#include "core/profile.h"
#include "driver/fmc.h"
#include "driver/motor.h"
#include "driver/time.h"
#include "flight/filter.h"
#include "flight/gestures.h"
#include "flight/imu.h"
#include "flight/input.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/led.h"
#include "io/usb_configurator.h"
#include "io/vbat.h"
#include "motor.h"
#include "turtle_mode.h"
#include "util/cbor_helper.h"
#include "util/util.h"

// Throttle must drop below this value if arming feature is enabled for arming to take place.
// brushed mix increase will also not activate on the ground untill this threshold is passed during takeoff for safety and better staging behavior.
#define THROTTLE_SAFETY .10f

#ifndef IDLE_THR
#define IDLE_THR .001f // just enough to override motor stop at 0 throttle
#endif

FAST_RAM control_flags_t flags = {
    .arm_state = 0,
    .arm_safety = 1,
    .throttle_safety = 0,

    .in_air = 0,
    .on_ground = 1,

    .failsafe = 1,
    .lowbatt = 1,

    .rx_mode = RXMODE_BIND,
    .rx_ready = 0,

    .controls_override = 0,
    .motortest_override = 0,

    .usb_active = 0,
};

FAST_RAM control_state_t state = {
    .failsafe_time_ms = 0,

    .aux = {0},

    .loop_counter = 0,

    .vbat = 0.0,
    .vbat_filtered = 0.0,
    .vbat_filtered_decay = 4.2,
    .vbat_compensated = 4.2,

    .ibat = 0.0,
    .ibat_filtered = 0.0,

    .stick_calibration_wizard = STICK_WIZARD_INACTIVE,

    .rx_filter_hz = 0.0f,

    .rx_rssi = 0,
    .rx_status = 0,

    .lipo_cell_count = 1.0,

    .GEstG = {
        .axis = {0, 0, ACC_1G},
    },
};

motor_test_t motor_test = {
    .active = 0,
    .value = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF},
};

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(control_state_t)
STATE_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef STR_MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

static void control_flight_mode() {
  // flight control
  const vec3_t rates = input_rates_calc();

  if (rx_aux_on(AUX_LEVELMODE)) {

    // calculate roll / pitch error
    const vec3_t errorvect = input_stick_vector(state.rx_filtered.axis);

    // apply yaw from the top of the quad
    // yaw rotation vector
    const float yawerror[3] = {
        state.GEstG.pitch * rates.yaw,
        -state.GEstG.roll * rates.yaw,
        state.GEstG.yaw * rates.yaw,
    };

    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) { // racemode with angle behavior on roll ais
      if (state.GEstG.yaw < 0) {                              // acro on roll and pitch when inverted
        state.setpoint.roll = rates.roll;
        state.setpoint.pitch = rates.pitch;

        state.error.roll = rates.roll - state.gyro.roll;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      } else {
        // roll is leveled to max angle limit
        state.angleerror[0] = errorvect.roll;
        state.setpoint.roll = angle_pid(0) + yawerror[0];
        state.error.roll = state.setpoint.roll - state.gyro.roll;

        // pitch is acro
        state.setpoint.pitch = rates.pitch;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      }
      // yaw
      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yawerror[2] - state.gyro.yaw;

    } else if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { // racemode with horizon behavior on roll axis
      float inclinationRoll = state.attitude.roll;
      float inclinationPitch = state.attitude.pitch;
      float inclinationMax;
      if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)) {
        inclinationMax = fabsf(inclinationRoll);
      } else {
        inclinationMax = fabsf(inclinationPitch);
      }
      float angleFade;
      // constrains acroFade variable between 0 and 1
      if (inclinationMax <= HORIZON_ANGLE_TRANSITION) {
        angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
      } else {
        angleFade = 1;
      }
      float stickFade;
      float deflection = fabsf(state.rx_filtered.roll);
      if (deflection <= HORIZON_STICK_TRANSITION) {
        stickFade = deflection / HORIZON_STICK_TRANSITION;
      } else {
        stickFade = 1;
      }
      float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
      // apply acro to roll for inverted behavior
      if (state.GEstG.yaw < 0) {
        state.setpoint.roll = rates.roll;
        state.setpoint.pitch = rates.pitch;
        state.error.roll = rates.roll - state.gyro.roll;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
        state.angleerror[0] = errorvect.roll;
        // roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
        state.setpoint.roll = (angle_pid(0) + yawerror[0]) * (1.0f - fade) + fade * (rates.roll);
        state.error.roll = ((angle_pid(0) + yawerror[0] - state.gyro.roll) * (1 - fade)) + (fade * (rates.roll - state.gyro.roll));
        // pitch is acro
        state.setpoint.pitch = rates.pitch;
        state.error.pitch = rates.pitch - state.gyro.pitch;
      }

      // yaw
      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yawerror[2] - state.gyro.yaw;

    } else if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { // horizon overrites standard level behavior
      // pitch and roll
      for (int i = 0; i <= 1; i++) {
        float inclinationRoll = state.attitude.roll;
        float inclinationPitch = state.attitude.pitch;
        float inclinationMax;
        if (fabsf(inclinationRoll) >= fabsf(inclinationPitch)) {
          inclinationMax = fabsf(inclinationRoll);
        } else {
          inclinationMax = fabsf(inclinationPitch);
        }
        float angleFade;
        // constrains acroFade variable between 0 and 1
        if (inclinationMax <= HORIZON_ANGLE_TRANSITION) {
          angleFade = inclinationMax / HORIZON_ANGLE_TRANSITION;
        } else {
          angleFade = 1;
        }
        float stickFade;
        float deflection = fabsf(state.rx_filtered.axis[i]);
        if (deflection <= HORIZON_STICK_TRANSITION) {
          stickFade = deflection / HORIZON_STICK_TRANSITION;
        } else {
          stickFade = 1;
        }
        float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
        // apply acro to roll and pitch sticks for inverted behavior
        if (state.GEstG.yaw < 0) {
          state.setpoint.axis[i] = rates.axis[i];
          state.error.axis[i] = rates.axis[i] - state.gyro.axis[i];
        } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
          state.angleerror[i] = errorvect.axis[i];
          //  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
          state.setpoint.axis[i] = (angle_pid(i) + yawerror[i]) * (1.0f - fade) + fade * (rates.axis[i]);
          state.error.axis[i] = ((angle_pid(i) + yawerror[i] - state.gyro.axis[i]) * (1 - fade)) + (fade * (rates.axis[i] - state.gyro.axis[i]));
        }
      }
      // yaw
      state.setpoint.yaw = rates.yaw;
      state.error.yaw = yawerror[2] - state.gyro.yaw;

    } else { // standard level mode
      // roll, pitch and yaw
      for (uint32_t i = 0; i < 3; i++) {
        state.angleerror[i] = errorvect.axis[i];
        state.setpoint.axis[i] = angle_pid(i) + yawerror[i];
        state.error.axis[i] = state.setpoint.axis[i] - state.gyro.axis[i];
      }
    }
  } else { // rate mode
    state.setpoint.roll = rates.roll;
    state.setpoint.pitch = rates.pitch;
    state.setpoint.yaw = rates.yaw;

    state.error.roll = state.setpoint.roll - state.gyro.roll;
    state.error.pitch = state.setpoint.pitch - state.gyro.pitch;
    state.error.yaw = state.setpoint.yaw - state.gyro.yaw;
  }
}

void control() {
  if (rx_aux_on(AUX_TURTLE) && !rx_aux_on(AUX_MOTOR_TEST)) { // turtle active when aux high
    turtle_mode_start();
  } else {
    turtle_mode_cancel();
  }

  turtle_mode_update();

  bool motortest_usb = false;
  if (flags.usb_active && motor_test.active) {
    // enable motortest for usb
    flags.arm_state = 1;
    flags.on_ground = 0;
    flags.motortest_override = 1;
    // motor test overwrites turtle
    flags.controls_override = 0;
    motortest_usb = true;
  } else if (flags.arm_state && rx_aux_on(AUX_MOTOR_TEST)) {
    // enable motortest for switch
    flags.motortest_override = 1;
    // motor test overwrites turtle
    flags.controls_override = 0;
  } else if (!flags.turtle) {
    // disable motortest unless turtle is active
    flags.motortest_override = 0;
  }

  if (flags.controls_override) {
    state.rx_filtered = state.rx_override;
  }

  control_flight_mode();
  pid_calc();

  bool failsafe_lock = false;
  if (flags.failsafe) {
    // failsafe first occured, record time
    if (state.failsafe_time_ms == 0) {
      state.failsafe_time_ms = time_millis();
    }
    if ((time_millis() - state.failsafe_time_ms) > FAILSAFE_LOCK_TIME_MS) {
      failsafe_lock = true;
    }
  } else {
    state.failsafe_time_ms = 0;
    failsafe_lock = false;
  }

  static bool checked_prearm = false;
  if (rx_aux_on(AUX_ARMING) && !failsafe_lock) {
    // CONDITION: AUX_ARMING is high

    if (!checked_prearm && rx_aux_on(AUX_PREARM) && state.rx_filtered.throttle <= THROTTLE_SAFETY) {
      // CONDITION:  we have not checked prearm this arm cycle AND AUX_PREARM is high AND throttle is zeroed
      flags.arm_switch = 1;

      // we passed the throttle safety check
      flags.throttle_safety = 0;

      if (!flags.turtle_ready) {
        motor_set_direction(MOTOR_FORWARD);
      }
    } else if (!flags.arm_switch) {
      if (state.rx_filtered.throttle > THROTTLE_SAFETY) {
        // throw up throttle safety if we crossed the threshold
        flags.throttle_safety = 1;
      } else {
        // throw up arming safety if we didnt manage to arm
        flags.arm_safety = 1;
      }
    }

    checked_prearm = true;
  } else {
    flags.arm_switch = 0;
    checked_prearm = false;

    if (flags.rx_ready == 1) {
      // rx is bound and has been disarmed so clear binding while armed flag
      flags.arm_safety = 0;
    }

    // clear the throttle safety flag
    flags.throttle_safety = 0;
  }

  if (flags.arm_switch) {
    // CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)
    if ((flags.throttle_safety == 1) || (flags.arm_safety == 1)) {
      // override to disarmed state
      flags.arm_state = 0;
    } else {
      // arm the quad by setting armed state variable to 1
      flags.arm_state = 1;
    }
  } else {
    // CONDITION: switch is DISARMED

    // disarm the quad by setting armed state variable to zero
    flags.arm_state = 0;
  }

  // CONDITION: armed state variable is 0 so quad is DISARMED
  if (flags.arm_state == 0) {
    // override throttle to 0
    state.throttle = 0;

    // flag in air variable as NOT IN THE AIR for mix throttle increase safety
    flags.in_air = 0;
  } else {
    // CONDITION: armed state variable is 1 so quad is ARMED
    if (!rx_aux_on(AUX_IDLE_UP)) {
      // CONDITION: idle up is turned OFF
      if (state.rx_filtered.throttle < 0.05f) {
        // set a small dead zone where throttle is zero
        state.throttle = 0;
      } else {
        // map the remainder of the the active throttle region to 100%
        state.throttle = (input_throttle_calc(state.rx_filtered.throttle) - 0.05f) * 1.05623158f;
      }
    } else {
      // CONDITION: idle up is turned ON
      if (flags.controls_override) {
        // override is active, set throttle to input
        state.throttle = state.rx_filtered.throttle;
      } else {
        // throttle range is mapped from idle throttle value to 100%
        state.throttle = (float)IDLE_THR + input_throttle_calc(state.rx_filtered.throttle) * (1.0f - (float)IDLE_THR);
      }
    }

    if ((state.rx_filtered.throttle > THROTTLE_SAFETY) && (flags.in_air == 0)) {
      // change the state of in air flag when first crossing the throttle
      // safety value to indicate craft has taken off for mix increase safety
      flags.in_air = 1;
    }
  }

  if (flags.motortest_override) {
    motor_test_calc(motortest_usb, state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);
  } else if (!flags.arm_state || flags.failsafe || (state.throttle < 0.001f)) {
    // CONDITION: disarmed OR failsafe OR throttle off
    flags.on_ground = 1;

    // zero throttle to reflect motors being off
    state.throttle = 0;
    state.thrsum = 0;

    motor_set_all(MOTOR_OFF);
  } else {
    // motors on - normal flight
    flags.on_ground = 0;

    if (!flags.controls_override) {
      // only modify throttle for stick input.
      // leave override values raw

      if (profile.motor.throttle_boost > 0.0f) {
        state.throttle += (float)(profile.motor.throttle_boost) * throttlehpf(state.throttle);
        state.throttle = constrain(state.throttle, 0.0f, 1.0f);
      }
    }

    motor_mixer_calc(state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);
  }

#ifdef MOTOR_BEEPS
  if ((flags.usb_active == 0 && flags.rx_ready && flags.failsafe && (time_millis() - state.failsafe_time_ms) > MOTOR_BEEPS_TIMEOUT) ||
      (flags.on_ground && rx_aux_on(AUX_BUZZER_ENABLE))) {
    motor_beep();
  } else
#endif
  {
    motor_update();
  }
}
// end of control function
