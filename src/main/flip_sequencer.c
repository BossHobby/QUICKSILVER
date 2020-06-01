#include "flip_sequencer.h"

#include <math.h>

#include "control.h"
#include "drv_time.h"
#include "profile.h"

#ifdef BRUSHLESS_TARGET
// TODO: enable for brushed too
#define STANDARD_TURTLE
#endif

#define TURTLE_TIMEOUT 1e6 //1 second timeout for auto turtle

#define THROTTLE_UP 1.0
#define THROTTLE_HOVER 0.5
#define THROTTLE_EXIT 0.6

#define FLIP_RATE 1500
//time to up throttle in beginning stage
// 200ms
#define THROTTLE_UP_TIME 200e3
// end time in level mode
#define LEVEL_MODE_TIME 200e3
#define LEVEL_MODE_ANGLE -0

#define FLIP_TIMEOUT_TOTAL 1500e3
#define FLIP_TIMEOUT_STAGE1 500e3

// don't change below

#define STAGE_FLIP_NONE 0
#define STAGE_FLIP_START 1
#define STAGE_FLIP_THROTTLEUP 2
#define STAGE_FLIP_ROTATING 3
#define STAGE_FLIP_ROTATING_INVERTED 4
#define STAGE_FLIP_LEVELMODE 5
#define STAGE_FLIP_EXIT 6

int motortest_override;
int level_override = 0;

unsigned long fliptime;
int readytoflip = 0;
int flipstage = STAGE_FLIP_NONE;
unsigned int levelmodetime;
int flipindex = 0;
int flipdir = 0;

extern profile_t profile;

#ifdef STANDARD_TURTLE
extern int pwmdir;
#endif

void start_flip() {
#ifdef STANDARD_TURTLE
  if (!readytoflip && flags.onground) { //if not currently queued up for a turtle sequence and disarmed
    readytoflip = 1;                    //queue up for a turtle event
    flipstage = STAGE_FLIP_NONE;
  }
#endif

#ifdef AUTOMATED_FLIP //depreciated - needs a proper going through if it is to be restored
  if (readytoflip == 0 && !flags.onground) {
    readytoflip = 1;
    fliptime = gettime();
    flipstage = STAGE_FLIP_START;
  }

  flipindex = 0;
  flipdir = 0;
  if (fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1])) {
    flipindex = 1;
    if (state.rx.axis[1] > 0)
      flipdir = 1;
  } else if (state.rx.axis[0] > 0)
    flipdir = 1;
#endif
}

void flip_sequencer() {
#ifdef STANDARD_TURTLE
  if (!readytoflip) { //turtle can't be initiated without the all clear flag - hold control variables at 0 state
    if (flipstage != STAGE_FLIP_NONE) {
      pwmdir = FORWARD;              //forward pwmdir only once as its last state may be unknown from previously interrupted turtle event
      flags.binding_while_armed = 1; //just in case absolutely require that the quad be disarmed when turning off turtle mode with a started sequencer
    }
    flipstage = STAGE_FLIP_NONE;
    flags.controls_override = 0;
    motortest_override = 0;
    return; //turtle mode off or flying away from a successful turtle will return here
  }         // a disarmed quad with turtle mode on will continue past

  //  track the change of onground and flag a potential turtle trigger event only on disarmed to armed event.
  int turtle_trigger = 0;
  static int last_armed_state_turtle;
  if (rx_aux_on(AUX_ARMING) != last_armed_state_turtle) {
    last_armed_state_turtle = rx_aux_on(AUX_ARMING);
    if (rx_aux_on(AUX_ARMING)) //quad was just armed - set the turtle_trigger flag to ready
      turtle_trigger = 1;      //trigger will reinit to 0 next go round
  }

  if ((state.GEstG.axis[2] < 0) && turtle_trigger) { //begin the turtle sequence only once and with turtle_trigger flag ready and while upside down.
    flipstage = STAGE_FLIP_START;
  }

  if (state.GEstG.axis[2] > 0.5f && flipstage) { //exit the sequence if you failed to turtle, picked up the quad, and flipped it over your damn self
    flipstage = STAGE_FLIP_EXIT;
  }

  switch (flipstage) {
  case STAGE_FLIP_NONE:
    break;

  case STAGE_FLIP_START:
    flags.controls_override = 1;
    state.rx_override.axis[0] = 0;
    state.rx_override.axis[1] = 0;
    state.rx_override.axis[2] = 0;
    state.rx_override.axis[3] = 0;
    motortest_override = 1;
    pwmdir = REVERSE;
    flipindex = 0;
    flipdir = 0;
    if (fabsf(state.rx.axis[0]) > 0.5f || fabsf(state.rx.axis[1]) > 0.5f) {
      if (fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1])) {
        flipindex = 1;
        if (state.rx.axis[1] > 0)
          flipdir = 1;
        flipstage = STAGE_FLIP_ROTATING;
        fliptime = gettime();
      } else {
        if (state.rx.axis[0] > 0)
          flipdir = 1;
        flipstage = STAGE_FLIP_ROTATING;
        fliptime = gettime();
      }
    }
    break;

  case STAGE_FLIP_ROTATING:
    state.rx_override.throttle = profile.motor.turtle_throttle_percent / 100.0f;
    if (flipdir) {
      state.rx_override.axis[flipindex] = 1.0f;
    } else {
      state.rx_override.axis[flipindex] = -1.0f;
    }
    if (gettime() - fliptime > TURTLE_TIMEOUT)
      flipstage = STAGE_FLIP_START;
    if (state.GEstG.axis[2] > 0.50f)
      flipstage = STAGE_FLIP_EXIT;
    break;

  case STAGE_FLIP_EXIT:
    readytoflip = 0;
    flipstage = STAGE_FLIP_NONE;
    flags.controls_override = 0;
    motortest_override = 0;
    pwmdir = FORWARD;
    flags.binding_while_armed = 1;
    break;
  }
#endif

#ifdef AUTOMATED_FLIP
  if (!readytoflip)
    return;

  if (flags.onground)
    flipstage = STAGE_FLIP_EXIT;

  if (readytoflip && gettime() - fliptime > FLIP_TIMEOUT_TOTAL) {
    // abort after timeout second
    flipstage = STAGE_FLIP_EXIT;
  }

  float max_rate = 860.0f;
  if (profile.rate.mode == RATE_MODE_BETAFLIGHT) {
    max_rate = 200 * profile.rate.betaflight.rc_rate.roll * (1 / 1 - (float)profile.rate.betaflight.super_rate.roll);
  } else {
    max_rate = (profile.rate.silverware.max_rate.roll + profile.rate.silverware.max_rate.pitch) / 2;
  }

  switch (flipstage) {
  case STAGE_FLIP_NONE:

    break;

  case STAGE_FLIP_START:
    //
    flags.acro_override = 1;
    flags.controls_override = 1;

    state.rx_override.axis[1] = 0;
    state.rx_override.axis[2] = 0;
    state.rx_override.throttle = THROTTLE_UP;

    if (state.GEstG.axis[2] < 0) {
      // flip initiated inverted
      if (flipdir)
        state.rx_override.axis[flipindex] = (float)FLIP_RATE / max_rate;
      else
        state.rx_override.axis[flipindex] = (float)-FLIP_RATE / max_rate;
      state.rx_override.throttle = THROTTLE_HOVER;
      flipstage = STAGE_FLIP_ROTATING_INVERTED;
    }
    flipstage = STAGE_FLIP_THROTTLEUP;

    break;

  case STAGE_FLIP_THROTTLEUP:

    if (gettime() - fliptime > THROTTLE_UP_TIME) {
      if (flipdir)
        state.rx_override.axis[flipindex] = (float)FLIP_RATE / max_rate;
      else
        state.rx_override.axis[flipindex] = (float)-FLIP_RATE / max_rate;
      state.rx_override.throttle = THROTTLE_UP;
      flipstage = STAGE_FLIP_ROTATING;
    }

    break;

  case STAGE_FLIP_ROTATING:
    if (gettime() - fliptime > FLIP_TIMEOUT_STAGE1 + THROTTLE_UP_TIME) {
      // abort
      flipstage = STAGE_FLIP_EXIT;
    }
    if (state.GEstG.axis[2] < 0) {
      //we are inverted
      state.rx_override.throttle = THROTTLE_HOVER;
      flipstage = STAGE_FLIP_ROTATING_INVERTED;
    }
    break;

  case STAGE_FLIP_ROTATING_INVERTED:
    if (state.GEstG.axis[2] > 0) {

      //we no longer inverted
      levelmodetime = gettime();

      state.rx_override.throttle = THROTTLE_EXIT;
      flags.acro_override = 0;
      level_override = 1;
      flipstage = STAGE_FLIP_LEVELMODE;
    }
    break;

  case STAGE_FLIP_LEVELMODE:
    // allow control in other axis at this point
    state.rx_override.axis[0] = state.rx.axis[0];
    state.rx_override.axis[1] = state.rx.axis[1];
    state.rx_override.axis[2] = state.rx.axis[2];

    if (flipdir)
      state.rx_override.axis[flipindex] = (float)LEVEL_MODE_ANGLE / profile.rate.level_max_angle;
    else
      state.rx_override.axis[flipindex] = (float)-LEVEL_MODE_ANGLE / profile.rate.level_max_angle;
    if (gettime() - levelmodetime > LEVEL_MODE_TIME)
      flipstage = STAGE_FLIP_EXIT;
    break;

  case STAGE_FLIP_EXIT:
    readytoflip = 0;
    flipstage = STAGE_FLIP_NONE;
    flags.acro_override = 0;
    level_override = 0;
    flags.controls_override = 0;
    break;

  default:
    flipstage = STAGE_FLIP_EXIT;
    break;
  }
#endif
}
