#include "flight/gestures.h"

#include <math.h>

#include "core/project.h"
#include "driver/time.h"
#include "flight/control.h"
#include "osd/render.h"

#define STICKMAX 0.7f
#define STICKCENTER 0.2f

#define GMACRO_LEFT (state.rx.roll < -STICKMAX || state.rx.yaw < -STICKMAX)
#define GMACRO_RIGHT (state.rx.roll > STICKMAX || state.rx.yaw > STICKMAX)
#define GMACRO_XCENTER (fabsf(state.rx.roll) < STICKCENTER && fabsf(state.rx.yaw) < STICKCENTER)

#define GMACRO_DOWN (state.rx.pitch < -STICKMAX)
#define GMACRO_UP (state.rx.pitch > STICKMAX)

#define GMACRO_PITCHCENTER (fabsf(state.rx.pitch) < STICKCENTER)

#define GESTURE_CENTER 0
#define GESTURE_CENTER_IDLE 12
#define GESTURE_LEFT 1
#define GESTURE_RIGHT 2
#define GESTURE_DOWN 3
#define GESTURE_UP 4
#define GESTURE_OTHER 127
#define GESTURE_LONG 255

#define GESTURETIME_MIN 50e3
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 700e3
#define GESTURETIME_IDLE_OSD 100e3

#define GSIZE 7
#define OSD_GSIZE 3

static uint8_t gbuffer[GSIZE];

static const uint8_t commands[GESTURE_MAX][GSIZE] = {
    [GESTURE_NONE] = {},

    // flash gestures
    [GESTURE_DDD] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER},
    [GESTURE_UUU] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER},

    // Gesture aux
    [GESTURE_LLD] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER},
    [GESTURE_RRD] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER},

    // Enter OSD
    [GESTURE_RRR] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER},
    // Refresh OSD
    [GESTURE_LRL] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER},
};

static const uint8_t osd_commands[OSD_INPUT_MAX][OSD_GSIZE] = {
    [OSD_INPUT_NONE] = {},
    [OSD_INPUT_UP] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER},
    [OSD_INPUT_DOWN] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER},
    [OSD_INPUT_RIGHT] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER},
    [OSD_INPUT_LEFT] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER},
};

static bool gesture_check_command(const uint8_t input[], const uint8_t command[]) {
  if (osd_state.screen != OSD_SCREEN_REGULAR) {
    for (int i = 0; i < OSD_GSIZE; i++) {
      if (input[i] != command[OSD_GSIZE - i - 1])
        return false;
    }
  } else {
    for (int i = 0; i < GSIZE; i++) {
      if (input[i] != command[GSIZE - i - 1])
        return false;
    }
  }
  return true;
}

static int32_t gesture_sequence(int32_t current_gesture) {
  if (current_gesture == gbuffer[0]) {
    return GESTURE_NONE;
  }

  if (osd_state.screen != OSD_SCREEN_REGULAR) {
    for (int i = OSD_GSIZE - 1; i >= 1; i--) {
      gbuffer[i] = gbuffer[i - 1];
    }

    gbuffer[0] = current_gesture;

    for (uint32_t i = 1; i < OSD_INPUT_MAX; i++) {
      if (gesture_check_command(gbuffer, osd_commands[i])) {
        gbuffer[1] = GESTURE_OTHER;
        return i;
      }
    }
  } else {
    for (int i = GSIZE - 1; i >= 1; i--) {
      gbuffer[i] = gbuffer[i - 1];
    }

    gbuffer[0] = current_gesture;

    for (uint32_t i = 1; i < GESTURE_MAX; i++) {
      if (gesture_check_command(gbuffer, commands[i])) {
        gbuffer[1] = GESTURE_OTHER;
        return i;
      }
    }
  }

  return GESTURE_NONE;
}

int32_t gestures_detect() {
  static int32_t gesture_start;
  static int32_t last_gesture;
  static int32_t current_gesture;
  static uint32_t gesture_time;

  if (!flags.on_ground) {
    current_gesture = GESTURE_OTHER;
    last_gesture = GESTURE_OTHER;
    return 0;
  }

  if (GMACRO_XCENTER && GMACRO_PITCHCENTER) {
    gesture_start = GESTURE_CENTER;
  } else if (GMACRO_LEFT && GMACRO_PITCHCENTER) {
    gesture_start = GESTURE_LEFT;
  } else if (GMACRO_RIGHT && GMACRO_PITCHCENTER) {
    gesture_start = GESTURE_RIGHT;
  } else if (GMACRO_DOWN && GMACRO_XCENTER) {
    gesture_start = GESTURE_DOWN;
  } else if (GMACRO_UP && GMACRO_XCENTER) {
    gesture_start = GESTURE_UP;
  }

  const uint32_t time = time_micros();
  if (gesture_start != last_gesture) {
    gesture_time = time;
  }

  if (time - gesture_time > GESTURETIME_MIN) {
    const uint32_t gesture_time_idle = osd_state.screen != OSD_SCREEN_REGULAR ? GESTURETIME_IDLE_OSD : GESTURETIME_IDLE;
    if ((gesture_start == GESTURE_CENTER) && (time - gesture_time > gesture_time_idle)) {
      current_gesture = GESTURE_CENTER_IDLE;
    } else if (time - gesture_time > GESTURETIME_MAX) {
      if ((gesture_start != GESTURE_OTHER))
        current_gesture = GESTURE_LONG;
    } else {
      current_gesture = gesture_start;
    }
  }

  last_gesture = gesture_start;

  return gesture_sequence(current_gesture);
}
