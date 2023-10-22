#include "core/scheduler.h"

#include <stdbool.h>
#include <string.h>

#include "core/debug.h"
#include "core/looptime.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "tasks.h"
#include "util/cbor_helper.h"

#define TASK_AVERAGE_SAMPLES 32
#define TASK_RUNTIME_REDUCTION 0.75
#define TASK_RUNTIME_MARGIN 1.25
#define TASK_RUNTIME_BUFFER 10

static FAST_RAM uint32_t task_queue_size = 0;
static FAST_RAM task_t *task_queue[TASK_MAX];
static FAST_RAM task_t *active_task = NULL;

static bool task_queue_contains(task_t *task) {
  for (uint32_t i = 0; i < task_queue_size; i++) {
    if (task_queue[i] == task) {
      return true;
    }
  }
  return false;
}

static bool task_queue_push(task_t *task) {
  if (task_queue_size >= TASK_MAX || task_queue_contains(task)) {
    return false;
  }
  for (uint32_t i = 0; i < TASK_MAX; i++) {
    if (task_queue[i] != NULL && task_queue[i]->priority <= task->priority) {
      continue;
    }

    memcpy(task_queue + i + 1, task_queue + i, (task_queue_size - i) * sizeof(task_t *));
    task_queue[i] = task;
    task_queue_size++;
    return true;
  }
  return false;
}

static inline bool task_should_run(const uint32_t start_cycles, uint8_t task_mask, task_t *task) {
  if ((task_mask & task->mask) == 0) {
    // task shall not run in this firmware state
    return false;
  }

  const int32_t time_left = US_TO_CYCLES(state.looptime_autodetect - TASK_RUNTIME_BUFFER) - (time_cycles() - start_cycles);
  if (task->priority != TASK_PRIORITY_REALTIME && task->runtime_worst > time_left) {
    // we dont have any time left this loop and task is not realtime
    task->runtime_worst *= TASK_RUNTIME_REDUCTION;
    return false;
  }

  return true;
}

static inline void task_run(task_t *task) {
  const volatile uint32_t start = time_cycles();

  task->flags = 0;
  active_task = task;
  task->func();
  active_task = NULL;

  const volatile uint32_t time_taken = time_cycles() - start;
  task->runtime_current = time_taken;

  if (task->flags & TASK_FLAG_SKIP_STATS) {
    return;
  }

  if (time_taken < task->runtime_min) {
    task->runtime_min = time_taken;
  }

  task->runtime_avg_sum -= task->runtime_avg;
  task->runtime_avg_sum += time_taken;
  task->runtime_avg = task->runtime_avg_sum / TASK_AVERAGE_SAMPLES;

  if (time_taken > task->runtime_max) {
    task->runtime_max = time_taken;
  }

  if (task->runtime_worst < (task->runtime_avg * TASK_RUNTIME_MARGIN)) {
    task->runtime_worst = task->runtime_avg * TASK_RUNTIME_MARGIN;
  }
}

static inline uint8_t scheduler_task_mask() {
  uint8_t task_mask = TASK_MASK_DEFAULT;
  if (flags.in_air || flags.arm_state) {
    task_mask |= TASK_MASK_IN_AIR;
  } else {
    task_mask |= TASK_MASK_ON_GROUND;
  }
  return task_mask;
}

void task_reset_runtime() {
  if (active_task != NULL) {
    active_task->flags |= TASK_FLAG_SKIP_STATS;
  }
  looptime_reset();
}

void scheduler_init() {
  looptime_init();

  for (uint32_t i = 0; i < TASK_MAX; i++) {
    task_queue_push(&tasks[i]);
  }
}

void scheduler_run() {
  looptime_reset();

  while (1) {
    const volatile uint32_t cycles = time_cycles();
    const uint8_t task_mask = scheduler_task_mask();
    for (uint32_t i = 0; i < task_queue_size; i++) {
      task_t *task = task_queue[i];
      if (task_should_run(cycles, task_mask, task)) {
        task_run(task);
      }
    }

    looptime_update();
  }
}

#ifdef DEBUG

#define ENCODE_CYCLES(val)                                  \
  {                                                         \
    const uint32_t us = CYCLES_TO_US(val);                  \
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &us)); \
  }

cbor_result_t cbor_encode_task_stats(cbor_value_t *enc) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array_indefinite(enc));

  for (uint32_t i = 0; i < TASK_MAX; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "name"));
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, tasks[i].name));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "current"));
    ENCODE_CYCLES(tasks[i].runtime_current)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "min"));
    ENCODE_CYCLES(tasks[i].runtime_min)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "avg"));
    ENCODE_CYCLES(tasks[i].runtime_avg)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max"));
    ENCODE_CYCLES(tasks[i].runtime_max)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "worst"));
    ENCODE_CYCLES(tasks[i].runtime_worst)

    CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

#endif