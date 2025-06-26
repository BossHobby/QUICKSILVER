#include "core/scheduler.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "core/debug.h"
#include "core/looptime.h"
#include "driver/time.h"
#include "flight/control.h"
#include "io/simulator.h"
#include "io/usb_configurator.h"
#include "tasks.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#define TASK_AVERAGE_SAMPLES 32
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

static FORCE_INLINE bool task_should_run(const uint32_t start_cycles, uint8_t task_mask, task_t *task, uint32_t task_id) {
  if ((task_mask & task->mask) == 0) {
    // task shall not run in this firmware state
    return false;
  }
  if (task->period_cycles > 0 && (start_cycles - task->last_time) < task->period_cycles) {
    return false;
  }

  const int32_t time_left = US_TO_CYCLES(state.looptime_autodetect - TASK_RUNTIME_BUFFER) - (time_cycles() - start_cycles);
  if (task->priority != TASK_PRIORITY_REALTIME && task->runtime_worst > time_left) {
    // we dont have any time left this loop and task is not realtime

    // Simple skip penalty using bit shift (faster than float multiply)
    task->runtime_worst = (task->runtime_worst * 3) >> 2; // 3/4 = 0.75

#ifdef DEBUG
    // Inline metrics update for skipped task
    task->metric_skip_count++;
    task->metric_consecutive_skips++;
    if (task->metric_consecutive_skips > task->metric_max_consecutive_skips) {
      task->metric_max_consecutive_skips = task->metric_consecutive_skips;
    }
#endif

    return false;
  }

  return true;
}

static FORCE_INLINE void task_run(task_t *task, uint32_t task_id) {
  const volatile uint32_t start = time_cycles();

  task->flags = 0;
  active_task = task;
  task->func();
  active_task = NULL;

  task->last_time = time_cycles();
  const volatile uint32_t time_taken = task->last_time - start;
  task->runtime_current = time_taken;

  if (state.loop_counter < 100) {
    // skip first couple of loops
    return;
  }

  if (task->flags & TASK_FLAG_SKIP_STATS) {
    return;
  }

  // Use bit shift for division (faster)
  task->runtime_avg_sum -= task->runtime_avg;
  task->runtime_avg_sum += time_taken;
  task->runtime_avg = task->runtime_avg_sum >> 5; // Divide by 32 (TASK_AVERAGE_SAMPLES)

  if (time_taken > task->runtime_max) {
    task->runtime_max = time_taken;
  }

  // Update exponential moving average of peaks for smooth P95 estimation
  // Track high values (above average) to estimate P95
  if (time_taken > task->runtime_avg) {
    // This value is above average
    if (task->runtime_peak_ema == 0) {
      // First initialization - start with this value
      task->runtime_peak_ema = time_taken;
    } else {
      // Update EMA based on how far above current estimate this value is
      if (time_taken > task->runtime_peak_ema) {
        // Value is above our estimate - pull it up faster (1/8 weight)
        task->runtime_peak_ema = ((task->runtime_peak_ema * 7) + time_taken) >> 3;
      } else {
        // Value is below our estimate - pull it down slower (1/32 weight)
        task->runtime_peak_ema = ((task->runtime_peak_ema * 31) + time_taken) >> 5;
      }
    }
  }
  // Always apply slow decay to forget old peaks (511/512 ≈ 0.2% per sample)
  if (task->runtime_peak_ema > task->runtime_avg) {
    task->runtime_peak_ema = (task->runtime_peak_ema * 511) >> 9;
  }

#ifdef DEBUG
  // Inline metrics update for executed task
  task->metric_consecutive_skips = 0;

  // Check for overrun
  if (time_taken > task->runtime_worst) {
    task->metric_overrun_count++;
  }

  // Update variance (simplified Welford's algorithm)
  if (state.loop_counter >= 100) {
    const float delta = time_taken - task->metric_mean_acc;
    task->metric_mean_acc += delta / (state.loop_counter - 100 + 1);
    const float delta2 = time_taken - task->metric_mean_acc;
    task->metric_m2 += delta * delta2;

    if (state.loop_counter > 100) {
      task->metric_variance = task->metric_m2 / (state.loop_counter - 100);
    }
  }
#endif

  // Use smooth P95 estimate for worst case estimation
  if (task->runtime_peak_ema > 0 && state.loop_counter > 500) {
    // Use the smooth P95 estimate with a small safety margin
    // 1.1x margin: multiply by 9 and shift by 3 = 1.125x
    const uint32_t stable_worst = (task->runtime_peak_ema * 9) >> 3;

    // Always update to ensure we track the smooth P95
    task->runtime_worst = stable_worst;
  } else {
    // Integer approximations for startup margins
    // 1.2 ≈ 6/5, 1.25 = 5/4 (was 1.1 which was too tight)
    const uint32_t margin_mult = (state.loop_counter < 450) ? 6 : 5;
    const uint32_t margin_shift = (state.loop_counter < 450) ? 2 : 2;
    const uint32_t margined = (task->runtime_avg * margin_mult) >> margin_shift;

    if (task->runtime_worst < margined) {
      task->runtime_worst = margined;
    }
  }
}

static FORCE_INLINE uint8_t scheduler_task_mask() {
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
    simulator_update();

    const volatile uint32_t cycles = time_cycles();
    const uint8_t task_mask = scheduler_task_mask();
    for (uint32_t i = 0; i < task_queue_size; i++) {
      task_t *task = task_queue[i];
      // Pass task index to avoid lookup later
      uint32_t task_id = task - tasks; // Pointer arithmetic to get index
      if (task_should_run(cycles, task_mask, task, task_id)) {
        task_run(task, task_id);
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

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "avg"));
    ENCODE_CYCLES(tasks[i].runtime_avg)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max"));
    ENCODE_CYCLES(tasks[i].runtime_max)

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "worst"));
    ENCODE_CYCLES(tasks[i].runtime_worst)

    // Add percentile data (using smooth P95 estimate)
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "percentile_95"));
    ENCODE_CYCLES(tasks[i].runtime_peak_ema)

    // Add variability metrics
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "stddev"));
    ENCODE_CYCLES((uint32_t)sqrtf(tasks[i].metric_variance))

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "cv_percent"));
    {
      const float cv = (tasks[i].runtime_avg > 0) ? (sqrtf(tasks[i].metric_variance) / tasks[i].runtime_avg) * 100.0f : 0.0f;
      CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &cv));
    }

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "skips"));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &tasks[i].metric_skip_count));

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "max_skips"));
    {
      const uint32_t max_skips = tasks[i].metric_max_consecutive_skips;
      CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &max_skips));
    }

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "overruns"));
    CBOR_CHECK_ERROR(res = cbor_encode_uint32_t(enc, &tasks[i].metric_overrun_count));

    // Note: Removed jitter_max and histogram for simplicity

    CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  }

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));

  return res;
}

#endif