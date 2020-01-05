/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <inttypes.h>
#include <math.h>

#include "control.h"
#include "defines.h"
#include "drv_fmc.h"
#include "drv_fmc2.h"
#include "drv_motor.h"
#include "drv_time.h"
#include "filter.h"
#include "flip_sequencer.h"
#include "gestures.h"
#include "led.h"
#include "pid.h"
#include "profile.h"
#include "sixaxis.h"
#include "util.h"

float throttle;
int idle_state;
extern int armed_state;
extern int in_air;
extern int arming_release;
extern int binding_while_armed;
extern int rx_ready;

extern float rx[];
extern float rx_filtered[];
extern float gyro[3];
extern int failsafe;
extern float pidoutput[PIDNUMBER];
extern float setpoint[3];

extern float angleerror[];
extern float attitude[];

int onground = 1;
int onground_long = 1;

float thrsum;

float error[PIDNUMBER];
float motormap(float input);

float yawangle;
uint8_t throttle_safety;
//extern float looptime;

extern int ledcommand;
//extern int ledblink;

extern float apid(int x);

#ifdef NOMOTORS
// to maintain timing or it will be optimized away
float tempx[4];
#endif

unsigned long timecommand = 0;

extern int motortest_override;
extern int controls_override;
extern float rx_override[];
extern int acro_override;

float overthrottlefilt = 0;
float underthrottlefilt = 0;

extern profile_t profile;

static float calcBFRatesRad(int axis) {
#define SETPOINT_RATE_LIMIT 1998.0f
#define RC_RATE_INCREMENTAL 14.54f

  float rcRate, superExpo;
  if (axis == ROLL) {
    rcRate = profile.rate.betaflight.rc_rate.roll;
    superExpo = profile.rate.betaflight.super_rate.roll;
  } else if (axis == PITCH) {
    rcRate = profile.rate.betaflight.rc_rate.pitch;
    superExpo = profile.rate.betaflight.super_rate.pitch;
  } else {
    rcRate = profile.rate.betaflight.rc_rate.yaw;
    superExpo = profile.rate.betaflight.super_rate.yaw;
  }
  if (rcRate > 2.0f) {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
  }
  const float rcCommandfAbs = rx_filtered[axis] > 0 ? rx_filtered[axis] : -rx_filtered[axis];
  float angleRate = 200.0f * rcRate * rx_filtered[axis];
  if (superExpo) {
    const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * superExpo), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
  }
  return constrainf(angleRate, -SETPOINT_RATE_LIMIT, SETPOINT_RATE_LIMIT) * (float)DEGTORAD;
}

void control(void) {
#ifdef INVERTED_ENABLE
  extern int pwmdir;
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

  if (controls_override) {
    for (int i = 0; i < 3; i++) {
      rx_filtered[i] = rx_override[i];
    }
  }
#endif

  // flight control
  float rates[3];

  // high-low rates switch
  float rate_multiplier = 1.0;
  if (rx_aux_on(AUX_HIGH_RATES) <= 0) {
    rate_multiplier = profile.rate.low_rate_mulitplier;
  }

  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    rates[0] = rate_multiplier * calcBFRatesRad(0);
    rates[1] = rate_multiplier * calcBFRatesRad(1);
    rates[2] = rate_multiplier * calcBFRatesRad(2);
  } else {
    rates[0] = rate_multiplier * rx_filtered[0] * profile.rate.silverware.max_rate.roll * DEGTORAD;
    rates[1] = rate_multiplier * rx_filtered[1] * profile.rate.silverware.max_rate.pitch * DEGTORAD;
    rates[2] = rate_multiplier * rx_filtered[2] * profile.rate.silverware.max_rate.yaw * DEGTORAD;
  }

  if (rx_aux_on(AUX_LEVELMODE) && !acro_override) {
    extern void stick_vector(float rx_input[], float maxangle);
    extern float errorvect[]; // level mode angle error calculated by stick_vector.c
    extern float GEstG[3];    // gravity vector for yaw feedforward
    float yawerror[3] = {0};  // yaw rotation vector
    // calculate roll / pitch error
    stick_vector(rx_filtered, 0);
    // apply yaw from the top of the quad
    yawerror[0] = GEstG[1] * rates[2];
    yawerror[1] = -GEstG[0] * rates[2];
    yawerror[2] = GEstG[2] * rates[2];

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
      if (GEstG[2] < 0) {                                     // acro on roll and pitch when inverted
        error[0] = rates[0] - gyro[0];
        error[1] = rates[1] - gyro[1];
      } else {
        //roll is leveled to max angle limit
        angleerror[0] = errorvect[0];
        error[0] = apid(0) + yawerror[0] - gyro[0];
        //pitch is acro
        error[1] = rates[1] - gyro[1];
      }
      // yaw
      error[2] = yawerror[2] - gyro[2];

    } else if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { //racemode with horizon behavior on roll axis
      float inclinationRoll = attitude[0];
      float inclinationPitch = attitude[1];
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
      float deflection = fabsf(rx_filtered[0]);
      if (deflection <= HORIZON_STICK_TRANSITION) {
        stickFade = deflection / HORIZON_STICK_TRANSITION;
      } else {
        stickFade = 1;
      }
      float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
      // apply acro to roll for inverted behavior
      if (GEstG[2] < 0) {
        error[0] = rates[0] - gyro[0];
        error[1] = rates[1] - gyro[1];
      } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
        angleerror[0] = errorvect[0];
        // roll angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
        error[0] = ((apid(0) + yawerror[0] - gyro[0]) * (1 - fade)) + (fade * (rates[0] - gyro[0]));
        //pitch is acro
        error[1] = rates[1] - gyro[1];
      }

      // yaw
      error[2] = yawerror[2] - gyro[2];

    } else if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON)) { //horizon overrites standard level behavior
      //pitch and roll
      for (int i = 0; i <= 1; i++) {
        float inclinationRoll = attitude[0];
        float inclinationPitch = attitude[1];
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
        float deflection = fabsf(rx_filtered[i]);
        if (deflection <= HORIZON_STICK_TRANSITION) {
          stickFade = deflection / HORIZON_STICK_TRANSITION;
        } else {
          stickFade = 1;
        }
        float fade = (stickFade * (1 - HORIZON_SLIDER)) + (HORIZON_SLIDER * angleFade);
        // apply acro to roll and pitch sticks for inverted behavior
        if (GEstG[2] < 0) {
          error[i] = rates[i] - gyro[i];
        } else { // apply a transitioning mix of acro and level behavior inside of stick HORIZON_TRANSITION point and full acro beyond stick HORIZON_TRANSITION point
          angleerror[i] = errorvect[i];
          //  angle strength fades out as sticks approach HORIZON_TRANSITION while acro stength fades in according to value of acroFade factor
          error[i] = ((apid(i) + yawerror[i] - gyro[i]) * (1 - fade)) + (fade * (rates[i] - gyro[i]));
        }
      }
      // yaw
      error[2] = yawerror[2] - gyro[2];

    } else { //standard level mode
      // pitch and roll
      for (int i = 0; i <= 1; i++) {
        angleerror[i] = errorvect[i];
        error[i] = apid(i) + yawerror[i] - gyro[i];
      }
      // yaw
      error[2] = yawerror[2] - gyro[2];
    }
  } else { // rate mode

    setpoint[0] = rates[0];
    setpoint[1] = rates[1];
    setpoint[2] = rates[2];

    for (int i = 0; i < 3; i++) {
      error[i] = setpoint[i] - gyro[i];
    }
  }

#ifdef YAW_FIX
  {
    rotateErrors();
    pid(0);
    pid(1);
    pid(2);
  }
#else
  {
    pid(0);
    pid(1);
    pid(2);
  }
#endif

#ifndef THROTTLE_SAFETY
#define THROTTLE_SAFETY .15f
#endif

  if (((rx[3] > THROTTLE_SAFETY) && (arming_release == 0)) && rx_aux_on(AUX_ARMING)) { //this is needed for osd warnings but could be better used below too
    throttle_safety = 1;
  } else {
    throttle_safety = 0;
  }

  if (!rx_aux_on(AUX_ARMING)) { // 						CONDITION: switch is DISARMED
    armed_state = 0;            // 												disarm the quad by setting armed state variable to zero
    if (rx_ready == 1)
      binding_while_armed = 0;                                                                //                        rx is bound and has been disarmed so clear binding while armed flag
  } else {                                                                                    // 						CONDITION: switch is ARMED
    if (((rx[3] > THROTTLE_SAFETY) && (arming_release == 0)) || (binding_while_armed == 1)) { //				   CONDITION: (throttle is above safety limit and ARMING RELEASE FLAG IS NOT CLEARED) OR (bind just took place with transmitter armed)
      armed_state = 0;                                                                        //                         	 				override to disarmed state and rapid blink the leds
      ledcommand = 1;
    } else {              //            					 CONDITION: quad is being armed in a safe state
      armed_state = 1;    //                      					  arm the quad by setting armed state variable to 1
      arming_release = 1; //                       						clear the arming release flag - the arming release flag being cleared
    }                     //											 						is what stops the quad from automatically disarming again the next time
  }                       //											 						throttle is raised above the safety limit

  if (!rx_aux_on(AUX_IDLE_UP)) {
    idle_state = 0;
  } else {
    idle_state = 1;
  }

#ifndef IDLE_THR
#define IDLE_THR .001f //just enough to override motor stop at 0 throttle
#endif

  if (armed_state == 0) {  // CONDITION: armed state variable is 0 so quad is DISARMED
    throttle = 0;          //						override throttle to 0
    in_air = 0;            //						flag in air variable as NOT IN THE AIR for mix throttle increase safety
    arming_release = 0;    //						arming release flag is set to not cleared to reactivate the throttle safety limit for the next arming event
  } else {                 // CONDITION: armed state variable is 1 so quad is ARMED
    if (idle_state == 0) { //            CONDITION: idle up is turned OFF
      if (rx[3] < 0.05f) {
        throttle = 0; //   											set a small dead zone where throttle is zero and
        in_air = 0;   //												deactivate mix increase 3 since throttle is off
      } else {
        throttle = (rx[3] - 0.05f) * 1.05623158f; //                        map the remainder of the the active throttle region to 100%
        in_air = 1;
      }                                                              //												activate mix increase since throttle is on
    } else {                                                         //						CONDITION: idle up is turned ON
      throttle = (float)IDLE_THR + rx[3] * (1.0f - (float)IDLE_THR); //            						throttle range is mapped from idle throttle value to 100%
      if ((rx[3] > THROTTLE_SAFETY) && (in_air == 0))
        in_air = 1; //            						change the state of in air flag when first crossing the throttle
    }               //            						safety value to indicate craft has taken off for mix increase safety
  }

#ifdef STICK_TRAVEL_CHECK //This feature completely disables throttle and allows visual feedback if control inputs reach full throws
  //Stick endpoints check tied to aux channel stick gesture
  if (rx_aux_on(AUX_TRAVEL_CHECK)) {
    throttle = 0;
    if ((rx[0] <= -0.99f) || (rx[0] >= 0.99f) || (rx[1] <= -0.99f) || (rx[1] >= 0.99f) || (rx[2] <= -0.99f) || (rx[2] >= 0.99f) || (rx[3] <= 0.0f) || (rx[3] >= 0.99f)) {
      ledcommand = 1;
    }
  }
#endif

  // turn motors off if throttle is off and pitch / roll sticks are centered
  if ((armed_state == 0) || failsafe || (throttle < 0.001f && (!ENABLESTIX || !onground_long || rx_aux_on(AUX_LEVELMODE) || (fabsf(rx[ROLL]) < (float)ENABLESTIX_TRESHOLD && fabsf(rx[PITCH]) < (float)ENABLESTIX_TRESHOLD && fabsf(rx[YAW]) < (float)ENABLESTIX_TRESHOLD)))) { // motors off

    if (onground_long) {
      if (gettime() - onground_long > ENABLESTIX_TIMEOUT) {
        onground_long = 0;
      }
    }

    for (int i = 0; i <= 3; i++) {
      motor_set(i, 0);
#ifdef MOTOR_FILTER
      // reset the motor filter
      motorfilter(0, i);
#endif
    }

#ifdef MOTOR_BEEPS
    extern void motorbeep(void);
    motorbeep();
#endif

    throttle = 0; //zero out throttle so it does not come back on as idle up value if enabled
    onground = 1;
    thrsum = 0;

  } else { // motors on - normal flight

    onground = 0;
    onground_long = gettime();

    float mix[4];

    if (profile.motor.throttle_boost > 0.0f) {
      throttle += (float)(profile.motor.throttle_boost) * throttlehpf(throttle);
      if (throttle < 0)
        throttle = 0;
      if (throttle > 1.0f)
        throttle = 1.0f;
    }

    if (controls_override) { // change throttle in flip mode
      throttle = rx_override[3];
    }

    // throttle angle compensation
#ifdef AUTO_THROTTLE
    if (rx_aux_on(AUX_LEVELMODE)) {
      //float autothrottle = fastcos(attitude[0] * DEGTORAD) * fastcos(attitude[1] * DEGTORAD);
      extern float GEstG[];
      float autothrottle = GEstG[2];
      float old_throttle = throttle;
      if (autothrottle <= 0.5f)
        autothrottle = 0.5f;
      throttle = throttle / autothrottle;
      // limit to 90%
      if (old_throttle < 0.9f)
        if (throttle > 0.9f)
          throttle = 0.9f;

      if (throttle > 1.0f)
        throttle = 1.0f;
    }
#endif

#ifdef LVC_LOWER_THROTTLE

#ifdef SWITCHABLE_FEATURE_2
    extern float vbatt_comp;
    extern float vbattfilt;
    extern int flash_feature_2;
    static float throttle_i = 0.0f;
    float throttle_p = 0.0f;
    if (flash_feature_2 == 1) {
      // can be made into a function
      if (vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
        throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) * (float)LVC_LOWER_THROTTLE_KP;
      // can be made into a function
      if (vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE)
        throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) * (float)LVC_LOWER_THROTTLE_KP;

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

      throttle -= throttle_p + throttle_i;
    } else {
      //do nothing - feature is disabled via stick gesture
    }
#else
    extern float vbatt_comp;
    extern float vbattfilt;
    static float throttle_i = 0.0f;
    float throttle_p = 0.0f;
    // can be made into a function
    if (vbattfilt < (float)LVC_LOWER_THROTTLE_VOLTAGE_RAW)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE_RAW - vbattfilt) * (float)LVC_LOWER_THROTTLE_KP;
    // can be made into a function
    if (vbatt_comp < (float)LVC_LOWER_THROTTLE_VOLTAGE)
      throttle_p = ((float)LVC_LOWER_THROTTLE_VOLTAGE - vbatt_comp) * (float)LVC_LOWER_THROTTLE_KP;

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

    throttle -= throttle_p + throttle_i;
#endif
#endif

    if (profile.motor.invert_yaw) {
      pidoutput[2] = -pidoutput[2];
    }

#ifdef INVERTED_ENABLE
    if (pwmdir == REVERSE) {
      // inverted flight

      mix[MOTOR_FR] = throttle + pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW]; // FR
      mix[MOTOR_FL] = throttle - pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW]; // FL
      mix[MOTOR_BR] = throttle + pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW]; // BR
      mix[MOTOR_BL] = throttle - pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW]; // BL

    } else
#endif
    {
      // normal mixer
      mix[MOTOR_FR] = throttle - pidoutput[ROLL] - pidoutput[PITCH] + pidoutput[YAW]; // FR
      mix[MOTOR_FL] = throttle + pidoutput[ROLL] - pidoutput[PITCH] - pidoutput[YAW]; // FL
      mix[MOTOR_BR] = throttle - pidoutput[ROLL] + pidoutput[PITCH] - pidoutput[YAW]; // BR
      mix[MOTOR_BL] = throttle + pidoutput[ROLL] + pidoutput[PITCH] + pidoutput[YAW]; // BL
    }

    // we invert again cause it's used by the pid internally (for limit)
    if (profile.motor.invert_yaw) {
      pidoutput[2] = -pidoutput[2];
    }

    for (int i = 0; i <= 3; i++) {
#ifdef MOTOR_FILTER
      mix[i] = motorfilter(mix[i], i);
#endif

#ifdef MOTOR_FILTER2_ALPHA
      float motorlpf(float in, int x);
      mix[i] = motorlpf(mix[i], i);
#endif

#ifdef MOTOR_KAL
      float motor_kalman(float in, int x);
      mix[i] = motor_kalman(mix[i], i);
#endif

      if (profile.motor.torque_boost > 0.0f) {
        float motord(float in, int x);
        mix[i] = motord(mix[i], i);
      }
    }

    //********************************MIXER SCALING***********************************************************

#ifdef BRUSHED_TARGET
#define BRUSHED_MIX_SCALING
#endif

#if defined(BRUSHLESS_TARGET) || defined(BRUSHLESS_MIX_SCALING)
#ifdef BRUSHLESS_MIX_SCALING
#undef BRUSHED_MIX_SCALING
#endif
#ifndef AIRMODE_STRENGTH
#define AIRMODE_STRENGTH 1.0f //  Most amount of power that can be added for Airmode
#endif
#ifndef CLIPPING_LIMIT
#define CLIPPING_LIMIT 1.0f //  Most amount of power that can be pulled before clipping
#endif

    static int mixScaling;
    if (onground)
      mixScaling = 0;
    // only enable once really in the air
    else
      mixScaling = in_air;
    if (mixScaling) {
      //ledcommand=1;
      float minMix = 1000.0f;
      float maxMix = -1000.0f;
      for (int i = 0; i < 4; i++) {
        if (mix[i] < minMix)
          minMix = mix[i];
        if (mix[i] > maxMix)
          maxMix = mix[i];
        if (minMix < (-AIRMODE_STRENGTH))
          minMix = (-AIRMODE_STRENGTH);
        if (maxMix > (1 + CLIPPING_LIMIT))
          maxMix = (1 + CLIPPING_LIMIT);
      }
      float mixRange = maxMix - minMix;
      float reduceAmount = 0.0f;
      if (mixRange > 1.0f) {
        float scale = 1.0f / mixRange;
        for (int i = 0; i < 4; i++)
          mix[i] *= scale;
        minMix *= scale;
        reduceAmount = minMix;
      } else {
        if (maxMix > 1.0f)
          reduceAmount = maxMix - 1.0f;
        else if (minMix < 0.0f)
          reduceAmount = minMix;
      }
      if (reduceAmount != 0.0f)
        for (int i = 0; i < 4; i++)
          mix[i] -= reduceAmount;
    }
#endif

#ifdef BRUSHED_MIX_SCALING

// options for mix throttle lowering if enabled
// 0 - 100 range ( 100 = full reduction / 0 = no reduction )
#ifndef MIX_THROTTLE_REDUCTION_PERCENT
#define MIX_THROTTLE_REDUCTION_PERCENT 10
#endif

// limit reduction and increase to this amount ( 0.0 - 1.0)
// 0.0 = no action
// 0.5 = reduce up to 1/2 throttle
//1.0 = reduce all the way to zero
#ifndef MIX_THROTTLE_REDUCTION_MAX
#define MIX_THROTTLE_REDUCTION_MAX 0.5
#endif

#ifndef MIX_MOTOR_MAX
#define MIX_MOTOR_MAX 1.0f
#endif

    //throttle reduction
    float overthrottle = 0;
    float underthrottle = 0.001f;

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

    //Brushed airmode - throttle increase

#ifndef MIX_THROTTLE_INCREASE_MAX
#define MIX_THROTTLE_INCREASE_MAX 0.2f
#endif
    if (in_air == 1) {
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

    //********************************MOTOR OUTPUT***********************************************************

    thrsum = 0; //reset throttle sum for voltage monitoring logic in main loop

    //Begin for-loop to send motor commands
    for (int i = 0; i <= 3; i++) {

      //***********************Motor Test Logic
#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)
#if defined(MOTORS_TO_THROTTLE) && !defined(MOTORS_TO_THROTTLE_MODE)
#undef MOTOR_MIN_COMMAND
#endif
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
      if ((rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE)) || (motortest_override)) {
#endif
        mix[i] = throttle;
        if (i == MOTOR_FL && (rx_filtered[ROLL] > 0.5f || rx_filtered[PITCH] < -0.5f)) {
          mix[i] = 0;
        }
        if (i == MOTOR_BL && (rx_filtered[ROLL] > 0.5f || rx_filtered[PITCH] > 0.5f)) {
          mix[i] = 0;
        }
        if (i == MOTOR_FR && (rx_filtered[ROLL] < -0.5f || rx_filtered[PITCH] < -0.5f)) {
          mix[i] = 0;
        }
        if (i == MOTOR_BR && (rx_filtered[ROLL] < -0.5f || rx_filtered[PITCH] > 0.5f)) {
          mix[i] = 0;
        }
#if defined(MOTORS_TO_THROTTLE_MODE) && !defined(MOTORS_TO_THROTTLE)
      }
#endif

// flash leds in valid throttle range
#ifdef MOTORS_TO_THROTTLE
      ledcommand = 1;
#warning "MOTORS TEST MODE"
#endif
#endif
      //***********************End Motor Test Logic

      //***********************Min Motor Command Logic

#ifdef BRUSHLESS_TARGET
// do nothing - idle set by DSHOT
#else
      if (profile.motor.digital_idle) {
        float motor_min_value = (float)profile.motor.digital_idle * 0.01f;
        if (mix[i] < 0)
          mix[i] = 0; //Clip all mixer values into 0 to 1 range before remapping
        if (mix[i] > 1)
          mix[i] = 1;
        mix[i] = motor_min_value + mix[i] * (1.0f - motor_min_value);
      }
#endif
      //***********************End Min Motor Command Logic

      //***********************Send Motor PWM Command Logic
#ifndef NOMOTORS
#ifndef MOTORS_TO_THROTTLE
      //normal mode
      motor_set(i, motormap(mix[i]));
#else
      // throttle test mode
      ledcommand = 1;
      motor_set(i, mix[i]);
#endif
#else
      // no motors mode ( anti-optimization)
#warning "NO MOTORS"
      tempx[i] = motormap(mix[i]);
#endif
      //***********************End Motor PWM Command Logic

      //***********************Clip mmixer outputs (if not already done) before applying calculating throttle sum
      if (mix[i] < 0)
        mix[i] = 0;
      if (mix[i] > 1)
        mix[i] = 1;
      thrsum += mix[i];
    }
    // end of for-loop to send motor PWM commands
    thrsum = thrsum / 4; //calculate throttle sum for voltage monitoring logic in main loop
  }
  // end motors on
}
// end of control function

#ifndef MOTOR_FILTER2_ALPHA
#define MOTOR_FILTER2_ALPHA 0.3
#endif

float motor_filt[4];
#if defined(MOTOR_FILTER2_ALPHA)
float motorlpf(float in, int x) {
  lpf(&motor_filt[x], in, 1 - MOTOR_FILTER2_ALPHA);
  return motor_filt[x];
}
#endif

//initial values for the kalman filter
float x_est_last[4];
float P_last[4];
//the noise in the system
const float Q = 0.02;
//the noise in the system ( variance -  squared )

#ifdef MOTOR_KAL
const float R = Q / (float)MOTOR_KAL;
#else
float R = 0.1;
#endif

float motor_kalman(float in, int x) {

  //do a prediction
  float x_temp_est = x_est_last[x];
  float P_temp = P_last[x] + Q;

  float K = P_temp * (1.0f / (P_temp + R));
  float x_est = x_temp_est + K * (in - x_temp_est);
  float P = (1 - K) * P_temp;

  //update our last's
  P_last[x] = P;
  x_est_last[x] = x_est;

  return x_est;
}

float motord(float in, int x) {
  float factor = profile.motor.torque_boost;
  static float lastratexx[4][4];

  float out = (+0.125f * in + 0.250f * lastratexx[x][0] - 0.250f * lastratexx[x][2] - (0.125f) * lastratexx[x][3]) * factor;

  lastratexx[x][3] = lastratexx[x][2];
  lastratexx[x][2] = lastratexx[x][1];
  lastratexx[x][1] = lastratexx[x][0];
  lastratexx[x][0] = in;

  return in + out;
}
