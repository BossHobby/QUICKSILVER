#pragma once

#include "project.h"

#ifdef DEBUG

typedef struct debug {
  float vbatt_comp;
  float adcfilt;
  float totaltime;
  float timefilt;
  float adcreffilt;
  float cpu_load;
  float max_cpu_load;
  uint32_t max_cpu_loop_number;
  uint32_t loops_between_max_cpu_load;
  float min_cpu_load;
  uint32_t min_cpu_loop_number;
  uint32_t loops_between_min_cpu_load;
} debug_type;

extern debug_type debug;

#endif