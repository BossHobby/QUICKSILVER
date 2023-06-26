#pragma once

#include <stdint.h>

#include "core/project.h"
#include "osd/render.h"

typedef enum {
  GESTURE_NONE,
  GESTURE_DDD,
  GESTURE_UUU,
  GESTURE_LLD,
  GESTURE_RRD,
  GESTURE_RRR,
  GESTURE_LRL,
#ifdef PID_GESTURE_TUNING
  GESTURE_UDU,
  GESTURE_UDD,
  GESTURE_UDR,
  GESTURE_UDL,
#endif
  GESTURE_MAX,
} gestures_t;

int32_t gestures_detect();
void gestures();
