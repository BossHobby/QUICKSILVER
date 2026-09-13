#pragma once

#include <stdint.h>

#include "util/vector.h"

enum gesture_command_t {
  GESTURE_NONE,
  GESTURE_CALIBRATE_SAVE,
  GESTURE_TOGGLE_BIND,
  GESTURE_OPEN_MENU,
  GESTURE_RESET_OSD,
  GESTURE_MENU_UP,
  GESTURE_MENU_DOWN,
  GESTURE_MENU_LEFT,
  GESTURE_MENU_RIGHT,
};

// Stick recognition. Disabled input and mode changes discard partial
// commands and require a neutral dwell before accepting another direction.
gesture_command_t gestures_detect(const vec4_t &sticks, bool enabled, bool menu, uint32_t now_us);
