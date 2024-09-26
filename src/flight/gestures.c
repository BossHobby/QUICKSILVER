#include "flight/gestures.h"

#include <math.h>

#include "core/flash.h"
#include "core/looptime.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/scheduler.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/pid.h"
#include "flight/sixaxis.h"
#include "io/led.h"
#include "osd/render.h"
#include "rx/rx.h"
#include "util/util.h"

typedef enum {
  GESTURE_CENTER = 0,
  GESTURE_LEFT = 1,
  GESTURE_RIGHT = 2,
  GESTURE_DOWN = 3,
  GESTURE_UP = 4,
  GESTURE_CENTER_IDLE = 12,
  GESTURE_OTHER = 127,
  GESTURE_LONG = 255,
} gesture_key_t;

#define STICKMAX 0.7f
#define STICKCENTER 0.2f

#define GMACRO_LEFT (state.rx.roll < -STICKMAX || state.rx.yaw < -STICKMAX)
#define GMACRO_RIGHT (state.rx.roll > STICKMAX || state.rx.yaw > STICKMAX)
#define GMACRO_XCENTER (fabsf(state.rx.roll) < STICKCENTER && fabsf(state.rx.yaw) < STICKCENTER)

#define GMACRO_DOWN (state.rx.pitch < -STICKMAX)
#define GMACRO_UP (state.rx.pitch > STICKMAX)

#define GMACRO_PITCHCENTER (fabsf(state.rx.pitch) < STICKCENTER)

#define GESTURETIME_MIN 50e3
#define GESTURETIME_MAX 500e3
#define GESTURETIME_IDLE 700e3
#define GESTURETIME_IDLE_OSD 100e3

#define GSIZE 7
#define OSD_GSIZE 3

static gesture_key_t gbuffer[GSIZE];

static const gesture_key_t commands[GESTURE_MAX][GSIZE] = {
    [GESTURE_NONE] = {},

    // flash gestures
    [GESTURE_DDD] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER, GESTURE_DOWN, GESTURE_CENTER},
    [GESTURE_UUU] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER, GESTURE_UP, GESTURE_CENTER},

    // Enter OSD
    [GESTURE_RRR] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER},
    // Refresh OSD
    [GESTURE_LRL] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER, GESTURE_RIGHT, GESTURE_CENTER, GESTURE_LEFT, GESTURE_CENTER},
};

static const gesture_key_t osd_commands[OSD_INPUT_MAX][OSD_GSIZE] = {
    [OSD_INPUT_NONE] = {},
    [OSD_INPUT_UP] = {GESTURE_CENTER_IDLE, GESTURE_UP, GESTURE_CENTER},
    [OSD_INPUT_DOWN] = {GESTURE_CENTER_IDLE, GESTURE_DOWN, GESTURE_CENTER},
    [OSD_INPUT_RIGHT] = {GESTURE_CENTER_IDLE, GESTURE_RIGHT, GESTURE_CENTER},
    [OSD_INPUT_LEFT] = {GESTURE_CENTER_IDLE, GESTURE_LEFT, GESTURE_CENTER},
};

static bool gesture_check_command(const gesture_key_t input[], const gesture_key_t command[]) {
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

static int32_t gesture_sequence(gesture_key_t current_gesture) {
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
  static gesture_key_t gesture_start;
  static gesture_key_t last_gesture;
  static gesture_key_t current_gesture;
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

void gestures() {
  if (!flags.on_ground || flags.gestures_disabled) {
    return;
  }

  static bool skip_calib = false;

  const int32_t command = gestures_detect();
  if (command == GESTURE_NONE) {
    return;
  }

  if (osd_state.screen != OSD_SCREEN_REGULAR) {
    return osd_handle_input(command);
  }

  switch (command) {
  case GESTURE_DDD: {
    // skip accel calibration if pid gestures used
    if (!skip_calib) {
      sixaxis_gyro_cal(); // for flashing lights
      sixaxis_acc_cal();
    } else {
      led_flash();
      skip_calib = false;
    }

    flash_save();
    flash_load();

    // reset loop time
    task_reset_runtime();
    break;
  }
  case GESTURE_UUU: {
    bind_storage.bind_saved = !bind_storage.bind_saved;
    skip_calib = true;
    led_flash();
    break;
  }
  case GESTURE_RRR: {
    osd_push_screen(OSD_SCREEN_MAIN_MENU);
    led_flash();
    break;
  }
  case GESTURE_LRL: {
    while (osd_pop_screen() != OSD_SCREEN_CLEAR)
      ;
    break;
  }
  }
}
