#pragma once

#include <stdint.h>

typedef enum {
  LOOPTIME_2K = 500,
  LOOPTIME_4K = 250,
  LOOPTIME_8K = 125,
} looptime_t;

void looptime_init();
void looptime_reset();
void looptime_update();
