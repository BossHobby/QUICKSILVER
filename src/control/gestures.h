#pragma once

#include <stdint.h>

#include "core/project.h"
#include "osd/render.h"

typedef enum {
  GESTURE_NONE,
  GESTURE_DDD,
  GESTURE_UUU,
  GESTURE_RRR,
  GESTURE_LRL,
  GESTURE_MAX,
} gestures_t;

int32_t gestures_detect();
void gestures();
