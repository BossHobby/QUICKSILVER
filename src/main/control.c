#include "control.h"

#include <math.h>
#include <stdint.h>

#include "angle_pid.h"
#include "drv_fmc.h"
#include "drv_motor.h"
#include "drv_time.h"
#include "filter.h"
#include "flip_sequencer.h"
#include "gestures.h"
#include "input.h"
#include "led.h"
#include "motor.h"
#include "pid.h"
#include "profile.h"
#include "sixaxis.h"
#include "usb_configurator.h"
#include "util.h"
#include "util/cbor_helper.h"

#ifndef THROTTLE_SAFETY
#define THROTTLE_SAFETY .15f
#endif

#ifndef IDLE_THR
#define IDLE_THR .001f //just enough to override motor stop at 0 throttle
#endif

#define ON_GROUND_LONG_TIMEOUT 1e6

control_flags_t flags = {
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
    .acro_override = 0,
    .motortest_override = 0,

    .usb_active = 0,
};

control_state_t state = {
    .aux = {0},

    .vbattfilt = 0.0,
    .vbattfilt_corr = 4.2,
    .vbatt_comp = 4.2,

    .rx_rssi = 0,
    .rx_status = 0,

    // voltage reference for vcc compensation
    .vreffilt = 1.0,

    .lipo_cell_count = 1.0,

    .GEstG = {
        .axis = {0, 0, ACC_1G},
    },
};

static uint8_t idle_state;
static uint8_t arming_release;
static uint32_t onground_long = 1;

extern int pwmdir;
extern int ledcommand;

extern profile_t profile;
extern usb_motor_test_t usb_motor_test;

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

void control(void) {
#ifdef INVERTED_ENABLE
//not supported yet
  if (rx_aux_on(AUX_FN_INVERTED))
    pwmdir = REVERSE;
  else
    pwmdir = FORWARD;
#endif

  rx_precalc();
  pid_precalc();

#ifndef DISABLE_FLIP_SEQUENCER
  if (rx_aux_on(AUX_TURTLE)) { //turtle active when aux high
    start_flip();
  } else {
    extern int readytoflip;
    readytoflip = 0; //reset the flip sequencer state variable with aux low
  }

  flip_sequencer();

  if (flags.controls_override) {
    for (int i = 0; i < 3; i++) {
      state.rx_filtered.axis[i] = state.rx_override.axis[i];
    }
  }
#endif

  // flight control
  float rates[3];

  input_rates_calc(rates);

  if (rx_aux_on(AUX_LEVELMODE) && !flags.acro_override) {
    float yawerror[3] = {0}; // yaw rotation vector

    // calculate roll / pitch error
    input_stick_vector(state.rx_filtered.axis, 0);

    // apply yaw from the top of the quad
    yawerror[0] = state.GEstG.axis[1] * rates[2];
    yawerror[1] = -state.GEstG.axis[0] * rates[2];
    yawerror[2] = state.GEstG.axis[2] * rates[2];

    // *************************************************************************
    //horizon modes tuning variables
    // *************************************************************************
    // 1.0 is pure angle based transition, 0.0 is pure stick defelction based transition, values inbetween are a mix of both.  Adjust from 0 to 1
    float HORIZON_SLIDER = 0.3f;
    //leveling transitions into acro below this angle - above this angle is all acro.  DO NOT SET ABOVE 85 DEGREES!
    float HORIZON_ANGLE_TRANSITION = 55.0f;
    //leveling transitions into acro below this stick position - beyond this stick position is all acro. Adjust from 0 to 1
    float HORIZON_STICK_TRANSITION = 0.95f;
    // *************************************************************************
    // *************************************************************************

    if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON)) { //racemode with angle behavior on roll ais
      if (state.GEstG.axis[2] < 0) {                          // acro on roll and pitch when inverted
        state.error.axis[0] = rates[0] - state.gyro.axis[0];
        state.error.axis[1] = rates[1] - state.gyro.axis[1];
      } else {
        //roll is leveled to max angle limit
        state.angleerror[0] = state.errorvect.axis[0];
        state.error.axis[0] = angle_pid(0) + yawerror[0] - state.gyro.axis[0];
        //pitch is acro
        state.error.axis[1] = rates[1] - state.gyro.axis[1];
      }
      // yaw
      state.error.axis[2] = yawerror[2] - state.gyro.axis[2];

    } else if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { //racemode with horizon behavior on roll axis
      float inclinationRoll = state.attitude.axis[0];
      float inclinationPitch = state.attitude.axis[1];
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
      float deflection = fabsf(state.rx_filtered.axis[0]);
      if (deflection <= HORIZON_STICK_TRANSITION) {
        stickFade = deflection / HORIZON_STICK_TRANSITION;
      } else {
        stickFade = 1;
      }
      float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
      // apply acro to roll for inverted behavior
      if (state.GEstG.axis[2] < 0) {
        state.error.axis[0] = rates[0] - state.gyro.axis[0];
        state.error.axis[1] = rates[1] - state.gyro.axis[1];
      } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
        state.angleerror[0] = state.errorvect.axis[0];
        // roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
        state.error.axis[0] = ((angle_pid(0) + yawerror[0] - state.gyro.axis[0]) * (1 - fade)) + (fade * (rates[0] - state.gyro.axis[0]));
        //pitch is acro
        state.error.axis[1] = rates[1] - state.gyro.axis[1];
      }

      // yaw
      state.error.axis[2] = yawerror[2] - state.gyro.axis[2];

    } else if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { //horizon overrites standard level behavior
      //pitch and roll
      for (int i = 0; i <= 1; i++) {
        float inclinationRoll = state.attitude.axis[0];
        float inclinationPitch = state.attitude.axis[1];
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
        if (state.GEstG.axis[2] < 0) {
          state.error.axis[i] = rates[i] - state.gyro.axis[i];
        } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
          state.angleerror[i] = state.errorvect.axis[i];
          //  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
          state.error.axis[i] = ((angle_pid(i) + yawerror[i] - state.gyro.axis[i]) * (1 - fade)) + (fade * (rates[i] - state.gyro.axis[i]));
        }
      }
      // yaw
      state.error.axis[2] = yawerror[2] - state.gyro.axis[2];

    } else { //standard level mode
      // pitch and roll
      for (int i = 0; i <= 1; i++) {
        state.angleerror[i] = state.errorvect.axis[i];
        state.error.axis[i] = angle_pid(i) + yawerror[i] - state.gyro.axis[i];
      }
      // yaw
      state.error.axis[2] = yawerror[2] - state.gyro.axis[2];
    }
  } else { // rate mode

    state.setpoint.axis[0] = rates[0];
    state.setpoint.axis[1] = rates[1];
    state.setpoint.axis[2] = rates[2];

    for (int i = 0; i < 3; i++) {
      state.error.axis[i] = state.setpoint.axis[i] - state.gyro.axis[i];
    }
  }

  {
#ifdef YAW_FIX
    rotateErrors();
#endif
    pid(0);
    pid(1);
    pid(2);
  }

  // CONDITION: switch is ARMED
  if (rx_aux_on(AUX_ARMING)) {
    // CONDITION: throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED
    if ((state.rx.throttle > THROTTLE_SAFETY) && (arming_release == 0)) {
      flags.throttle_safety = 1;
    } else {
      flags.throttle_safety = 0;
    }

    // CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)
    if ((flags.throttle_safety == 1) || (flags.arm_safety == 1)) {
      // override to disarmed state
      flags.arm_state = 0;

      // rapid blink the leds
      ledcommand = 1;
    } else {
      // CONDITION: quad is being armed in a safe state

      // arm the quad by setting armed state variable to 1
      flags.arm_state = 1;

      // clear the arming release flag - the arming release flag being cleared
      // is what stops the quad from automatically disarming again the next time
      // throttle is raised above the safety limit
      arming_release = 1;
    }
  } else {
    // CONDITION: switch is DISARMED

    // disarm the quad by setting armed state variable to zero
    flags.arm_state = 0;
    // clear the throttle safety flag
    flags.throttle_safety = 0;
    if (flags.rx_ready == 1) {
      // rx is bound and has been disarmed so clear binding while armed flag
      flags.arm_safety = 0;
    }
  }

  if (!rx_aux_on(AUX_IDLE_UP)) {
    idle_state = 0;
  } else {
    idle_state = 1;
  }

  // CONDITION: armed state variable is 0 so quad is DISARMED
  if (flags.arm_state == 0) {
    // override throttle to 0
    state.throttle = 0;

    // flag in air variable as NOT IN THE AIR for mix throttle increase safety
    flags.in_air = 0;

    // arming release flag is set to not cleared to reactivate the throttle safety limit for the next arming event
    arming_release = 0;

  } else {
    // CONDITION: armed state variable is 1 so quad is ARMED

    if (idle_state == 0) {
      // CONDITION: idle up is turned OFF

      if (state.rx.throttle < 0.05f) {
        // set a small dead zone where throttle is zero and
        state.throttle = 0;

        // deactivate mix increase 3 since throttle is off
        flags.in_air = 0;
      } else {
        // map the remainder of the the active throttle region to 100%
        state.throttle = (state.rx.throttle - 0.05f) * 1.05623158f;

        // activate mix increase since throttle is on
        flags.in_air = 1;
      }
    } else {
      // CONDITION: idle up is turned ON

      // throttle range is mapped from idle throttle value to 100%
      state.throttle = (float)IDLE_THR + state.rx.throttle * (1.0f - (float)IDLE_THR);

      if ((state.rx.throttle > THROTTLE_SAFETY) && (flags.in_air == 0)) {
        // change the state of in air flag when first crossing the throttle
        // safety value to indicate craft has taken off for mix increase safety
        flags.in_air = 1;
      }
    }
  }

#ifdef STICK_TRAVEL_CHECK //This feature completely disables throttle and allows visual feedback if control inputs reach full throws
  //Stick endpoints check tied to aux channel stick gesture
  if (rx_aux_on(AUX_TRAVEL_CHECK)) {
    state.throttle = 0;
    if ((state.rx.axis[0] <= -0.99f) || (state.rx.axis[0] >= 0.99f) ||
        (state.rx.axis[1] <= -0.99f) || (state.rx.axis[1] >= 0.99f) ||
        (state.rx.axis[2] <= -0.99f) || (state.rx.axis[2] >= 0.99f) ||
        (state.rx.axis[3] <= 0.0f) || (state.rx.axis[3] >= 0.99f)) {
      ledcommand = 1;
    }
  }
#endif

  if (usb_motor_test.active) {
    flags.arm_state = 1;
    flags.on_ground = 0;

    motor_mixer_calc(state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);
  } else if ((flags.arm_state == 0) || flags.failsafe || (state.throttle < 0.001f)) {
    // CONDITION: disarmed OR failsafe OR throttle off

    if (onground_long && (timer_micros() - onground_long > ON_GROUND_LONG_TIMEOUT)) {
      onground_long = 0;
    }

    // turn motors off
    motor_set_all(0);

#ifdef MOTOR_BEEPS
    motor_beep();
#endif

    state.throttle = 0; //zero out throttle so it does not come back on as idle up value if enabled
    flags.on_ground = 1;
    state.thrsum = 0;

  } else { // motors on - normal flight

    flags.on_ground = 0;
    onground_long = timer_micros();

    if (profile.motor.throttle_boost > 0.0f) {
      state.throttle += (float)(profile.motor.throttle_boost) * throttlehpf(state.throttle);
      if (state.throttle < 0)
        state.throttle = 0;
      if (state.throttle > 1.0f)
        state.throttle = 1.0f;
    }

    if (flags.controls_override) { // change throttle in flip mode
      state.throttle = state.rx_override.throttle;
    }

    // throttle angle compensation
#ifdef AUTO_THROTTLE
    if (rx_aux_on(AUX_LEVELMODE)) {
      //float autothrottle = fastcos(state.attitude.axis[0] * DEGTORAD) * fastcos(state.attitude.axis[1] * DEGTORAD);
      float autothrottle = state.GEstG.axis[2];
      float old_throttle = state.throttle;
      if (autothrottle <= 0.5f)
        autothrottle = 0.5f;
      state.throttle = state.throttle / autothrottle;
      // limit to 90%
      if (old_throttle < 0.9f)
        if (state.throttle > 0.9f)
          state.throttle = 0.9f;

      if (state.throttle > 1.0f)
        state.throttle = 1.0f;
    }
#endif

#ifdef LVC_LOWER_THROTTLE

#ifdef SWITCHABLE_FEATURE_2
    static float throttle_i = 0.0f;
    float throttle_p = 0.0f;
    if (flash_storage.flash_feature_2 == 1) {
      // can be made into a function
      if (state.vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
        throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - state.vbattfilt) * (float)LVC_LOWER_THROTTLE_KP;
      // can be made into a function
      if (state.vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE)
        throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - state.vbatt_comp) * (float)LVC_LOWER_THROTTLE_KP;

      if (throttle_p > 1.0f)
        throttle_p = 1.0f;

      if (throttle_p > 0) {
        throttle_i += throttle_p * 0.0001f; //ki
      } else
        throttle_i -= 0.001f; // ki on release

      if (throttle_i > 0.5f)
        throttle_i = 0.5f;
      if (throttle_i < 0.0f)
        throttle_i = 0.0f;

      state.throttle -= throttle_p + throttle_i;
    } else {
      //do nothing - feature is disabled via stick gesture
    }
#else
    static float throttle_i = 0.0f;
    float throttle_p = 0.0f;
    // can be made into a function
    if (state.vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - state.vbattfilt) * (float)LVC_LOWER_THROTTLE_KP;
    // can be made into a function
    if (state.vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - state.vbatt_comp) * (float)LVC_LOWER_THROTTLE_KP;

    if (throttle_p > 1.0f)
      throttle_p = 1.0f;

    if (throttle_p > 0) {
      throttle_i += throttle_p * 0.0001f; //ki
    } else
      throttle_i -= 0.001f; // ki on release

    if (throttle_i > 0.5f)
      throttle_i = 0.5f;
    if (throttle_i < 0.0f)
      throttle_i = 0.0f;

    state.throttle -= throttle_p + throttle_i;
#endif
#endif

    if (profile.motor.invert_yaw) {
      state.pidoutput.yaw = -state.pidoutput.yaw;
    }

    motor_mixer_calc(state.motor_mix.axis);
    motor_output_calc(state.motor_mix.axis);

    // we invert again cause it's used by the pid internally (for limit)
    if (profile.motor.invert_yaw) {
      state.pidoutput.yaw = -state.pidoutput.yaw;
    }
  }
  // end motors on
}
// end of control function
