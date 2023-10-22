#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "project.h"

typedef enum {
  TASK_GYRO,
  TASK_IMU,
  TASK_PID,
  TASK_RX,
  TASK_UTIL,
  TASK_BLACKBOX,
  TASK_OSD,
  TASK_VTX,
  TASK_USB,

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

  uint32_t runtime_current;
  uint32_t runtime_min;
  uint32_t runtime_avg;
  uint32_t runtime_worst;
  uint32_t runtime_max;

  uint32_t runtime_avg_sum;
} task_t;

#define CREATE_TASK(p_name, p_mask, p_priority, p_func) \
  {                                                     \
    .name = p_name,                                     \
    .mask = p_mask,                                     \
    .flags = 0,                                         \
    .priority = p_priority,                             \
    .func = p_func,                                     \
    .runtime_current = 0,                               \
    .runtime_min = UINT32_MAX,                          \
    .runtime_avg = 0,                                   \
    .runtime_worst = 0,                                 \
    .runtime_max = 0,                                   \
    .runtime_avg_sum = 0,                               \
  }

extern task_t tasks[TASK_MAX];