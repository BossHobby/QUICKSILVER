#include "core/scheduler.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "control/control.h"
#include "core/debug.h"
#include "core/looptime.h"
#include "driver/time.h"
#include "io/simulator.h"
#include "io/usb_configurator.h"
#include "tasks.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#define TASK_AVERAGE_SAMPLES 32
#define TASK_RUNTIME_BUFFER 10
#define TASK_STARVATION_SKIPS 32
static_assert(TASK_MAX < TASK_STARVATION_SKIPS);

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
  uint32_t i = 0;
  while (i < task_queue_size && task_queue[i]->priority <= task->priority) {
    i++;
  }
  memmove(task_queue + i + 1, task_queue + i, (task_queue_size - i) * sizeof(task_t *));
  task_queue[i] = task;
  task_queue_size++;
  return true;
}

static FORCE_INLINE bool task_should_run(uint32_t start_cycles, uint8_t task_mask, task_t *task, bool &forced_retry_used) {
  if ((task_mask & task->mask) == 0) {
    return false;
  }
  if (task->period_cycles > 0 && (start_cycles - task->last_time) < task->period_cycles) {
    return false;
  }
  if (task->priority == TASK_PRIORITY_REALTIME) {
    return true;
  }

  const uint32_t budget_cycles = US_TO_CYCLES(MAX(0.0f, state.looptime_autodetect - TASK_RUNTIME_BUFFER));
  const uint32_t elapsed_cycles = time_cycles() - start_cycles;
  if (elapsed_cycles < budget_cycles && task->runtime_worst <= budget_cycles - elapsed_cycles) {
    return true;
  }

  // One forced retry per loop. A serviced task waits another 32 skips, giving
  // all other tasks a turn before it can retry again (TASK_MAX < 32).
  if (task->runtime_skips >= TASK_STARVATION_SKIPS && !forced_retry_used) {
    forced_retry_used = true;
    return true;
  }
  if (task->runtime_skips < TASK_STARVATION_SKIPS) {
    task->runtime_skips++;
  }

#if defined(DEBUG) || defined(PIO_UNIT_TESTING)
  task->metric_skip_count++;
  task->metric_consecutive_skips++;
  task->metric_max_consecutive_skips = MAX(task->metric_max_consecutive_skips, task->metric_consecutive_skips);
#endif
  return false;
}

static FORCE_INLINE void task_run(task_t *task, uint32_t release_cycles) {
  const uint32_t start = time_cycles();
  const uint32_t start_us = time_micros();
  task->last_period_us = task->has_started ? start_us - task->last_start_us : 0;
  task->last_start_us = start_us;
  task->has_started = true;

  task->flags = 0;
  task->runtime_skips = 0;
#if defined(DEBUG) || defined(PIO_UNIT_TESTING)
  task->metric_consecutive_skips = 0;
#endif
  active_task = task;
  task->func();
  active_task = NULL;

  const uint32_t time_taken = time_cycles() - start;
  task->last_time = release_cycles;
  task->runtime_current = time_taken;

  if (state.loop_counter < 100 || (task->flags & TASK_FLAG_SKIP_STATS)) {
    return;
  }

  task->runtime_avg_sum -= task->runtime_avg;
  task->runtime_avg_sum += time_taken;
  task->runtime_avg = task->runtime_avg_sum / TASK_AVERAGE_SAMPLES;
  task->runtime_max = MAX(task->runtime_max, time_taken);

  // Track an EMA of above-average runtimes; this is not a percentile.
  if (time_taken > task->runtime_avg) {
    if (task->runtime_peak_ema == 0) {
      task->runtime_peak_ema = time_taken;
    } else if (time_taken > task->runtime_peak_ema) {
      task->runtime_peak_ema = ((task->runtime_peak_ema * 7) + time_taken) >> 3;
    } else {
      task->runtime_peak_ema = ((task->runtime_peak_ema * 31) + time_taken) >> 5;
    }
  }
  // Always apply slow decay to forget old peaks (511/512 ≈ 0.2% per sample)
  if (task->runtime_peak_ema > task->runtime_avg) {
    // Equivalent to floor(peak * 511 / 512), without overflowing the product.
    const uint32_t decay = (task->runtime_peak_ema >> 9) + ((task->runtime_peak_ema & 511) != 0);
    task->runtime_peak_ema -= decay;
  }

#if defined(DEBUG) || defined(PIO_UNIT_TESTING)
  // Check for overrun
  if (time_taken > task->runtime_worst) {
    task->metric_overrun_count++;
  }

  // Welford sample variance, counting only this task's accepted samples.
  if (task->metric_sample_count == UINT32_MAX) {
    task->metric_sample_count = 0;
    task->metric_mean_acc = 0;
    task->metric_m2 = 0;
  }
  task->metric_sample_count++;
  const float delta = time_taken - task->metric_mean_acc;
  task->metric_mean_acc += delta / task->metric_sample_count;
  const float delta2 = time_taken - task->metric_mean_acc;
  task->metric_m2 += delta * delta2;
  task->metric_variance = task->metric_sample_count > 1 ? task->metric_m2 / (task->metric_sample_count - 1) : 0;
#endif

  // Admission uses a heuristic peak estimate with a margin, not a WCET bound.
  if (task->runtime_peak_ema > 0 && state.loop_counter > 500) {
    task->runtime_worst = (task->runtime_peak_ema * 9) / 8; // 1.125x
  } else {
    // Startup margin: 1.5x initially, then 1.25x.
    const uint32_t margin_mult = (state.loop_counter < 450) ? 6 : 5;
    task->runtime_worst = MAX(task->runtime_worst, task->runtime_avg * margin_mult / 4);
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

static void scheduler_run_tasks(uint32_t cycles) {
  bool forced_retry_used = false;
  for (uint32_t i = 0; i < task_queue_size; i++) {
    task_t *task = task_queue[i];
    // Control can arm or disarm during this iteration.
    if (task_should_run(cycles, scheduler_task_mask(), task, forced_retry_used)) {
      task_run(task, cycles);
    }
  }
}

void scheduler_run() {
  looptime_reset();

  while (1) {
    const uint32_t cycles = looptime_update();
    simulator_update();
    scheduler_run_tasks(cycles);
  }
}

#ifdef PIO_UNIT_TESTING
static bool test_forced_retry_used;

void scheduler_test_begin_loop() {
  test_forced_retry_used = false;
}

bool scheduler_test_should_run(uint32_t start_cycles, uint8_t task_mask, task_t *task) {
  return task_should_run(start_cycles, task_mask, task, test_forced_retry_used);
}

void scheduler_test_run(task_t *task, uint32_t release_cycles) {
  task_run(task, release_cycles);
}

static void scheduler_test_set_tasks(task_t *input, uint32_t count) {
  task_queue_size = 0;
  for (uint32_t i = 0; i < count; i++) {
    task_queue_push(&input[i]);
  }
}

void scheduler_test_order_tasks(task_t *input, uint32_t count, task_t **output) {
  scheduler_test_set_tasks(input, count);
  memcpy(output, task_queue, task_queue_size * sizeof(task_t *));
  task_queue_size = 0;
}

void scheduler_test_run_tasks(task_t *input, uint32_t count, uint32_t cycles) {
  scheduler_test_set_tasks(input, count);
  scheduler_run_tasks(cycles);
  task_queue_size = 0;
}
#endif

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

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "runtime_peak"));
    ENCODE_CYCLES(tasks[i].runtime_peak_ema)

    // Add variability metrics
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "stddev"));
    ENCODE_CYCLES((uint32_t)sqrtf(tasks[i].metric_variance))

    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "cv_percent"));
    {
      const float cv = (tasks[i].metric_mean_acc > 0) ? (sqrtf(tasks[i].metric_variance) / tasks[i].metric_mean_acc) * 100.0f : 0.0f;
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
