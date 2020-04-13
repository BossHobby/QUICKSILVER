#include "motor.h"

#include "drv_motor.h"
#include "motorcurve.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef MOTORS_TO_THROTTLE
#warning "MOTORS TEST MODE"
#endif

float thrsum;

#ifdef NOMOTORS
#warning "NO MOTORS"
float tempx[4];
#endif

extern int motortest_override;

extern float throttle;
extern float rx_filtered[];

extern profile_t profile;

//********************************MOTOR OUTPUT***********************************************************
void motor_output_calc(float mix[4]) {
  thrsum = 0; //reset throttle sum for voltage monitoring logic in main loop

  //Begin for-loop to send motor commands
  for (int i = 0; i <= 3; i++) {

    //***********************Motor Test Logic
#if defined(MOTORS_TO_THROTTLE) || defined(MOTORS_TO_THROTTLE_MODE)

#if defined(MOTORS_TO_THROTTLE)
    motortest_override = 1;
#endif

    if (rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE) || motortest_override) {
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
    }

    // TODO: flash leds in valid throttle range ?
    // ledcommand = 1;
#endif

    //***********************Min Motor Command Logic
#if defined(BRUSHED_TARGET)
    if (profile.motor.digital_idle && !(rx_aux_on(AUX_MOTORS_TO_THROTTLE_MODE) || motortest_override)) {
      float motor_min_value = (float)profile.motor.digital_idle * 0.01f;
      //Clip all mixer values into 0 to 1 range before remapping
      mix[i] = constrainf(mix[i], 0, 1);
      mix[i] = motor_min_value + mix[i] * (1.0f - motor_min_value);
    }
#else
    // brushless: do nothing - idle set by DSHOT
#endif

    //***********************Send Motor PWM Command Logic
#ifndef NOMOTORS
    //normal mode
    motor_set(i, motormap(mix[i]));
#else
    // no motors mode
    // to maintain timing or it will be optimized away
    tempx[i] = motormap(mix[i]);
#endif

    // clip mixer outputs (if not already done) before applying calculating throttle sum
    mix[i] = constrainf(mix[i], 0, 1);
    thrsum += mix[i];
  }

  //calculate throttle sum for voltage monitoring logic in main loop
  thrsum = thrsum / 4;
}