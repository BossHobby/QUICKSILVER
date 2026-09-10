#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "project.h"

#include "control/control.h"
#include "driver/time.h"

typedef enum {
  TASK_GYRO,
  TASK_IMU,
  TASK_ATTITUDE,
  TASK_PID,
  TASK_RX,
  TASK_VBAT,
  TASK_NAV,
  TASK_UTIL,
  TASK_GESTURES,
  TASK_BLACKBOX,
  TASK_OSD,
  TASK_VTX,
  TASK_USB,
  TASK_GPS,

  TASK_MAX

} task_id_t;

typedef enum {
  TASK_PRIORITY_REALTIME,
  TASK_PRIORITY_HIGH,
  TASK_PRIORITY_MEDIUM,
  TASK_PRIORITY_LOW,
} task_priority_t;

typedef enum {
  TASK_MASK_DEFAULT = (0x1 << 0),
  TASK_MASK_ON_GROUND = (0x1 << 1),
  TASK_MASK_IN_AIR = (0x1 << 2),

  TASK_MASK_ALWAYS = 0xFF,
} task_mask_t;

typedef enum {
  TASK_FLAG_SKIP_STATS = (0x1 << 0),
} task_flag_t;

typedef void (*task_function_t)();
typedef bool (*task_poll_function_t)();

typedef struct {
  const char *name;

  uint8_t mask;
  uint32_t flags;
  task_priority_t priority;
  task_function_t func;
  uint32_t period_cycles;

  uint32_t last_time; // Loop-start cycles of the last execution, for periodic release.
  uint32_t last_start_us;
  uint32_t last_period_us; // Actual start-to-start interval; zero on first execution.
  uint32_t runtime_current;
  uint32_t runtime_avg;
  uint32_t runtime_worst;
  uint32_t runtime_max;
  // Consecutive budget skips; permits an occasional retry of a stale estimate.
  uint8_t runtime_skips;
  bool has_started;

  uint32_t runtime_avg_sum;

  // EMA of above-average runtimes, not a percentile or worst-case bound.
  uint32_t runtime_peak_ema;

#if defined(DEBUG) || defined(PIO_UNIT_TESTING)
  // Minimal metrics for debug builds only
  uint32_t metric_skip_count;
  uint32_t metric_overrun_count;
  uint8_t metric_consecutive_skips;
  uint8_t metric_max_consecutive_skips;

  // Per-task sample variance in cycles squared.
  uint32_t metric_sample_count;
  float metric_variance;
  float metric_mean_acc;
  float metric_m2;
#endif
} task_t;

#define CREATE_TASK(p_name, p_mask, p_priority, p_func, p_period_us) \
  {                                                                  \
      .name = p_name,                                                \
      .mask = p_mask,                                                \
      .flags = 0,                                                    \
      .priority = p_priority,                                        \
      .func = p_func,                                                \
      .period_cycles = US_TO_CYCLES(p_period_us),                    \
      .last_time = 0,                                                \
      .last_start_us = 0,                                            \
      .last_period_us = 0,                                           \
      .runtime_current = 0,                                          \
      .runtime_avg = 0,                                              \
      .runtime_worst = 0,                                            \
      .runtime_max = 0,                                              \
      .runtime_skips = 0,                                            \
      .has_started = false,                                          \
      .runtime_avg_sum = 0,                                          \
      .runtime_peak_ema = 0,                                         \
  }

extern task_t tasks[TASK_MAX];

// Nominal cadence for configuration, not elapsed time for integration.
static inline float task_get_period_us(task_id_t id) {
  const float period = CYCLES_TO_US(tasks[id].period_cycles);
  if (period > 0.0f)
    return period;
  return state.looptime_autodetect;
}

// Published before the task runs; includes delayed/skipped releases. Unsigned
// subtraction supports timer wrap, provided starts are less than ~71 min apart.
static inline uint32_t task_get_last_period_us(task_id_t id) {
  return tasks[id].last_period_us;
}
