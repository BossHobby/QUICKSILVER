#include "turtle_mode.h"

#include <math.h>

#include "drv_motor.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"

// TODO: enable for brushed too
#ifdef BRUSHLESS_TARGET

#define TURTLE_TIMEOUT 1000 // 1 second timeout for auto turtle

typedef enum {
  TURTLE_STAGE_IDLE,
  TURTLE_STAGE_START,
  TURTLE_STAGE_ROTATING,
  TURTLE_STAGE_EXIT,
} turtle_mode_stage_t;

static turtle_mode_stage_t turtle_state = TURTLE_STAGE_IDLE;
static bool turtle_ready = false;
static uint32_t turtle_time = 0;
static uint8_t turtle_axis = 0;
static uint8_t turtle_dir = 0;

void turtle_mode_start() {
  if (!turtle_ready && flags.on_ground) {
    turtle_ready = true;
    turtle_state = TURTLE_STAGE_IDLE;
  }
}

void turtle_mode_cancel() {
  turtle_ready = false;
}

void turtle_mode_update() {
  if (turtle_state > TURTLE_STAGE_IDLE && flags.arm_state == 1) {
    flags.turtle = 1;
  } else {
    flags.turtle = 0;
  }

  // turtle can't be initiated without the all clear flag - hold control variables at 0 state
  if (!turtle_ready) {
    if (turtle_state != TURTLE_STAGE_IDLE) {
      flags.arm_safety = 1; // just in case absolutely require that the quad be disarmed when turning off turtle mode with a started sequencer

      // force motors to forward, in case turtle was interrupted
      if (!motor_set_direction(MOTOR_FORWARD)) {
        return;
      }
    }

    // turtle mode off or flying away from a successful turtle will return here
    // a disarmed quad with turtle mode on will continue past
    turtle_state = TURTLE_STAGE_IDLE;
    flags.controls_override = 0;
    flags.motortest_override = 0;

    return;
  }

  if (state.GEstG.yaw > 0.5f && turtle_state) {
    // exit the sequence if you failed to turtle, picked up the quad, and flipped it over your damn self
    turtle_state = TURTLE_STAGE_EXIT;
  }

  switch (turtle_state) {
  case TURTLE_STAGE_IDLE: {
    static uint8_t last_armed_state_turtle = 0;
    if (flags.arm_switch != last_armed_state_turtle) {
      last_armed_state_turtle = flags.arm_switch;

      // quad was just armed and upside down, begin the turtle sequence
      if (flags.arm_switch && (state.GEstG.yaw < 0)) {
        turtle_state = TURTLE_STAGE_START;
      }
    }
    break;
  }
  case TURTLE_STAGE_START:
    flags.controls_override = 1;
    flags.motortest_override = 1;
    state.rx_override.axis[0] = 0;
    state.rx_override.axis[1] = 0;
    state.rx_override.yaw = 0;
    state.rx_override.axis[3] = 0;

    if (!motor_set_direction(MOTOR_REVERSE)) {
      // wait for the motor to sucessfully change
      break;
    }

    if (fabsf(state.rx.axis[0]) > 0.5f || fabsf(state.rx.axis[1]) > 0.5f) {
      if (fabsf(state.rx.axis[0]) < fabsf(state.rx.axis[1])) {
        turtle_axis = 1;

        if (state.rx.axis[1] > 0) {
          turtle_dir = 1;
        } else {
          turtle_dir = 0;
        }
      } else {
        turtle_axis = 0;

        if (state.rx.axis[0] > 0) {
          turtle_dir = 1;
        } else {
          turtle_dir = 0;
        }
      }

      turtle_state = TURTLE_STAGE_ROTATING;
      turtle_time = time_millis();
    }
    break;

  case TURTLE_STAGE_ROTATING:
    state.rx_override.throttle = profile.motor.turtle_throttle_percent / 100.0f;
    if (turtle_dir) {
      state.rx_override.axis[turtle_axis] = 1.0f;
    } else {
      state.rx_override.axis[turtle_axis] = -1.0f;
    }

    if (time_millis() - turtle_time > TURTLE_TIMEOUT) {
      turtle_state = TURTLE_STAGE_START;
    }

    if (state.GEstG.yaw > 0.50f) {
      turtle_state = TURTLE_STAGE_EXIT;
    }
    break;

  case TURTLE_STAGE_EXIT:
    flags.controls_override = 0;
    flags.motortest_override = 0;
    flags.arm_safety = 1;

    if (!motor_set_direction(MOTOR_FORWARD)) {
      // wait for the motor to sucessfully change
      break;
    }

    turtle_state = TURTLE_STAGE_IDLE;
    turtle_ready = false;
    break;
  }
}
#else
void turtle_mode_start() {}
void turtle_mode_cancel() {}

void turtle_mode_update() {}
#endif