#include "flip_sequencer.h"

#include <math.h>

#include "drv_motor.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"

#ifdef BRUSHLESS_TARGET
// TODO: enable for brushed too
#define STANDARD_TURTLE
#endif

#define TURTLE_TIMEOUT 1e6 // 1 second timeout for auto turtle

// don't change below

#define STAGE_FLIP_NONE 0
#define STAGE_FLIP_START 1
#define STAGE_FLIP_THROTTLEUP 2
#define STAGE_FLIP_ROTATING 3
#define STAGE_FLIP_ROTATING_INVERTED 4
#define STAGE_FLIP_LEVELMODE 5
#define STAGE_FLIP_EXIT 6

int level_override = 0;

uint32_t fliptime;
int readytoflip = 0;
int flipstage = STAGE_FLIP_NONE;
uint32_t levelmodetime;
int flipindex = 0;
int flipdir = 0;

extern profile_t profile;

#ifdef STANDARD_TURTLE
extern uint8_t pwmdir;
#endif

void start_flip() {
#ifdef STANDARD_TURTLE
  if (!readytoflip && flags.on_ground) { // if not currently queued up for a turtle sequence and disarmed
    readytoflip = 1;                     // queue up for a turtle event
    flipstage = STAGE_FLIP_NONE;
  }
#endif
}

void flip_sequencer() {
#ifdef STANDARD_TURTLE
  if (flipstage > 0 && flags.arm_state == 1)
    flags.turtle = 1;
  else
    flags.turtle = 0;
  if (!readytoflip) { // turtle can't be initiated without the all clear flag - hold control variables at 0 state
    if (flipstage != STAGE_FLIP_NONE) {
      pwmdir = FORWARD;     // forward pwmdir only once as its last state may be unknown from previously interrupted turtle event
      flags.arm_safety = 1; // just in case absolutely require that the quad be disarmed when turning off turtle mode with a started sequencer
    }
    flipstage = STAGE_FLIP_NONE;
    flags.controls_override = 0;
    flags.motortest_override = 0;
    return; // turtle mode off or flying away from a successful turtle will return here
  }         // a disarmed quad with turtle mode on will continue past

  //  track the change of on_ground and flag a potential turtle trigger event only on disarmed to armed event.
  int turtle_trigger = 0;
  static int last_armed_state_turtle;
  if (rx_aux_on(AUX_ARMING) != last_armed_state_turtle) {
    last_armed_state_turtle = rx_aux_on(AUX_ARMING);
    if (rx_aux_on(AUX_ARMING)) // quad was just armed - set the turtle_trigger flag to ready
      turtle_trigger = 1;      // trigger will reinit to 0 next go round
  }

  if ((state.GEstG.axis[2] < 0) && turtle_trigger) { // begin the turtle sequence only once and with turtle_trigger flag ready and while upside down.
    flipstage = STAGE_FLIP_START;
  }

  if (state.GEstG.axis[2] > 0.5f && flipstage) { // exit the sequence if you failed to turtle, picked up the quad, and flipped it over your damn self
    flipstage = STAGE_FLIP_EXIT;
  }

  switch (flipstage) {
  case STAGE_FLIP_NONE:
    break;

  case STAGE_FLIP_START:
    flags.controls_override = 1;
    flags.motortest_override = 1;
    state.rx_override.axis[0] = 0;
    state.rx_override.axis[1] = 0;
    state.rx_override.axis[2] = 0;
    state.rx_override.axis[3] = 0;
    pwmdir = REVERSE;
    flipindex = 0;
    flipdir = 0;
    if (fabsf(state.rx.axis[0]) > 0.5f || fabsf(state.rx.axis[1]) > 0.5f) {
      if (fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1])) {
        flipindex = 1;
        if (state.rx.axis[1] > 0)
          flipdir = 1;
        flipstage = STAGE_FLIP_ROTATING;
        fliptime = time_micros();
      } else {
        if (state.rx.axis[0] > 0)
          flipdir = 1;
        flipstage = STAGE_FLIP_ROTATING;
        fliptime = time_micros();
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
    if (time_micros() - fliptime > TURTLE_TIMEOUT)
      flipstage = STAGE_FLIP_START;
    if (state.GEstG.axis[2] > 0.50f)
      flipstage = STAGE_FLIP_EXIT;
    break;

  case STAGE_FLIP_EXIT:
    readytoflip = 0;
    flipstage = STAGE_FLIP_NONE;
    flags.controls_override = 0;
    flags.motortest_override = 0;
    pwmdir = FORWARD;
    flags.arm_safety = 1;
    break;
  }
#endif
}
