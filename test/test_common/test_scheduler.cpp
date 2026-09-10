#include <unity.h>

#include "core/tasks.h"
#include "core/scheduler.h"
#include "core/looptime.h"
#include "driver/time.h"

extern bool scheduler_test_should_run(uint32_t start_cycles, uint8_t task_mask, task_t *task);
extern void scheduler_test_run(task_t *task, uint32_t release_cycles);
extern void scheduler_test_begin_loop();
extern void scheduler_test_order_tasks(task_t *input, uint32_t count, task_t **output);
extern void scheduler_test_run_tasks(task_t *input, uint32_t count, uint32_t cycles);

static uint32_t task_runtime_us;

static void timed_task() {
  time_test_advance_us(task_runtime_us);
}

static task_t background_task(uint32_t runtime_us) {
  scheduler_test_begin_loop();
  state.looptime_autodetect = 125.0f;
  task_t task = {};
  task.mask = TASK_MASK_ALWAYS;
  task.priority = TASK_PRIORITY_LOW;
  task.runtime_worst = US_TO_CYCLES(runtime_us);
  return task;
}

void test_scheduler_rejects_exhausted_budget() {
  task_t task = background_task(20);
  time_test_set_us(130);
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));

  // Even an unmeasured task must not start at the reserved margin or beyond it.
  task.runtime_worst = 0;
  time_test_set_us(115);
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));
  time_test_set_us(116);
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));

  state.looptime_autodetect = 5.0f;
  time_test_set_us(0);
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));
}

void test_scheduler_preserves_runtime_estimate_on_skips() {
  task_t task = background_task(40);
  time_test_set_us(100); // Only 15 us remains after reserving the margin.
  for (uint32_t i = 0; i < 16; i++) {
    TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));
    TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(40), task.runtime_worst);
  }

  // An exact fit in a subsequent loop is still admitted.
  time_test_set_us(200);
  TEST_ASSERT_TRUE(scheduler_test_should_run(US_TO_CYCLES(125), TASK_MASK_IN_AIR, &task));
}

void test_scheduler_budget_handles_cycle_wrap() {
  task_t task = background_task(20);
  time_test_set_us(10);
  const uint32_t start_cycles = time_cycles() - US_TO_CYCLES(95);
  TEST_ASSERT_TRUE(scheduler_test_should_run(start_cycles, TASK_MASK_IN_AIR, &task));
  time_test_advance_us(1);
  TEST_ASSERT_FALSE(scheduler_test_should_run(start_cycles, TASK_MASK_IN_AIR, &task));
  time_test_advance_us(30);
  TEST_ASSERT_FALSE(scheduler_test_should_run(start_cycles, TASK_MASK_IN_AIR, &task));
}

void test_scheduler_realtime_respects_mask_and_period() {
  task_t task = background_task(40);
  task.priority = TASK_PRIORITY_REALTIME;
  time_test_set_us(130);
  TEST_ASSERT_TRUE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));

  task.mask = TASK_MASK_ON_GROUND;
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));
  task.mask = TASK_MASK_ALWAYS;
  task.period_cycles = US_TO_CYCLES(1000);
  TEST_ASSERT_FALSE(scheduler_test_should_run(0, TASK_MASK_IN_AIR, &task));
}

void test_scheduler_period_handles_cycle_wrap() {
  task_t task = background_task(20);
  task.period_cycles = US_TO_CYCLES(1000);
  time_test_set_us(10);
  const uint32_t start_cycles = time_cycles();
  task.last_time = start_cycles - US_TO_CYCLES(999);
  TEST_ASSERT_FALSE(scheduler_test_should_run(start_cycles, TASK_MASK_IN_AIR, &task));
  task.last_time = start_cycles - US_TO_CYCLES(1000);
  TEST_ASSERT_TRUE(scheduler_test_should_run(start_cycles, TASK_MASK_IN_AIR, &task));
}

void test_scheduler_recovers_after_runtime_spike() {
  task_t task = background_task(0);
  task.func = timed_task;
  const uint32_t saved_loop_counter = state.loop_counter;
  state.loop_counter = 501;

  task_runtime_us = 10;
  scheduler_test_run(&task, time_cycles());
  task_runtime_us = 100;
  scheduler_test_run(&task, time_cycles());
  const uint32_t spike_estimate = task.runtime_worst;
  TEST_ASSERT_GREATER_THAN_UINT32(US_TO_CYCLES(15), spike_estimate);

  task_runtime_us = 5;
  for (uint32_t i = 0; i < 32; i++) {
    const uint32_t start = time_cycles();
    time_test_advance_us(100);
    TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
    TEST_ASSERT_EQUAL_UINT32(spike_estimate, task.runtime_worst);
    time_test_advance_us(25);
  }

  const uint32_t start = time_cycles();
  time_test_advance_us(100);
  TEST_ASSERT_TRUE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
  scheduler_test_run(&task, start);
  TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(5), task.runtime_current);
  TEST_ASSERT_EQUAL_UINT8(0, task.runtime_skips);

  // If the estimate still exceeds the window, retries must remain spaced out.
  TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
  state.loop_counter = saved_loop_counter;
}

void test_scheduler_recovers_from_sustained_budget_exhaustion() {
  task_t task = background_task(40);
  task.func = timed_task;
  task_runtime_us = 5;
  for (uint32_t i = 0; i < 32; i++) {
    const uint32_t start = time_cycles();
    time_test_advance_us(118);
    TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
    TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(40), task.runtime_worst);
    time_test_advance_us(7);
  }

  // Sustained realtime load must not prevent the bounded retry.
  const uint32_t start = time_cycles();
  time_test_advance_us(118);
  TEST_ASSERT_TRUE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
  task.mask = TASK_MASK_ON_GROUND;
  TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
  task.mask = TASK_MASK_ALWAYS;
  task.last_time = start;
  task.period_cycles = US_TO_CYCLES(1000);
  TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
  task.period_cycles = 0;
  scheduler_test_run(&task, start);
  TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(5), task.runtime_current);
  TEST_ASSERT_EQUAL_UINT8(0, task.runtime_skips);

  // Execution restores budget enforcement, including beyond the deadline.
  time_test_advance_us(10);
  TEST_ASSERT_FALSE(scheduler_test_should_run(start, TASK_MASK_IN_AIR, &task));
}

void test_scheduler_limits_forced_retries_and_services_all_tasks() {
  task_t pending[TASK_MAX] = {};
  for (auto &task : pending) {
    task = background_task(40);
    task.func = timed_task;
    task.runtime_skips = 32;
  }
  task_runtime_us = 40;
  bool serviced[TASK_MAX] = {};
  for (uint32_t loop = 0; loop < TASK_MAX; loop++) {
    scheduler_test_begin_loop();
    const uint32_t start = time_cycles();
    time_test_advance_us(130);
    uint32_t runs = 0;
    for (uint32_t i = 0; i < TASK_MAX; i++) {
      if (scheduler_test_should_run(start, TASK_MASK_IN_AIR, &pending[i])) {
        TEST_ASSERT_FALSE(serviced[i]);
        serviced[i] = true;
        runs++;
        scheduler_test_run(&pending[i], start);
      }
    }
    TEST_ASSERT_EQUAL_UINT32(1, runs);
  }
  for (bool ran : serviced) {
    TEST_ASSERT_TRUE(ran);
  }
}

void test_scheduler_period_uses_loop_release_not_completion() {
  task_t task = background_task(10);
  task.func = timed_task;
  task.period_cycles = US_TO_CYCLES(1000);
  task_runtime_us = 10;
  time_test_set_us(1000);
  uint32_t release = time_cycles();
  time_test_advance_us(100); // Earlier tasks consume part of this iteration.
  TEST_ASSERT_TRUE(scheduler_test_should_run(release, TASK_MASK_IN_AIR, &task));
  scheduler_test_run(&task, release);
  TEST_ASSERT_EQUAL_UINT32(release, task.last_time);
  TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(10), task.runtime_current);

  time_test_set_us(2000);
  release = time_cycles();
  time_test_advance_us(100);
  TEST_ASSERT_TRUE(scheduler_test_should_run(release, TASK_MASK_IN_AIR, &task));

  // A missed release produces one execution, never a burst of catch-up work.
  time_test_set_us(5500);
  release = time_cycles();
  TEST_ASSERT_TRUE(scheduler_test_should_run(release, TASK_MASK_IN_AIR, &task));
  scheduler_test_run(&task, release);
  TEST_ASSERT_FALSE(scheduler_test_should_run(release, TASK_MASK_IN_AIR, &task));
}

static void timed_task_skip_stats() {
  timed_task();
  task_reset_runtime();
}

void test_scheduler_variance_counts_task_samples() {
  task_t task = background_task(10);
  task.func = timed_task;
  const uint32_t saved_loop_counter = state.loop_counter;
  for (uint32_t i = 0; i < 10; i++) {
    state.loop_counter = 1000 + i * 100;
    task_runtime_us = 10;
    scheduler_test_run(&task, time_cycles());
  }
  TEST_ASSERT_EQUAL_UINT32(10, task.metric_sample_count);
  TEST_ASSERT_EQUAL_FLOAT(US_TO_CYCLES(10), task.metric_mean_acc);
  TEST_ASSERT_EQUAL_FLOAT(0, task.metric_variance);

  task = background_task(10);
  task.func = timed_task;
  task_runtime_us = 10;
  scheduler_test_run(&task, time_cycles());
  task_runtime_us = 30;
  scheduler_test_run(&task, time_cycles());
  const float delta_cycles = US_TO_CYCLES(10);
  TEST_ASSERT_EQUAL_FLOAT(US_TO_CYCLES(20), task.metric_mean_acc);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 2 * delta_cycles * delta_cycles, task.metric_variance);

  task.func = timed_task_skip_stats;
  task_runtime_us = 1000;
  scheduler_test_run(&task, time_cycles());
  TEST_ASSERT_EQUAL_UINT32(2, task.metric_sample_count);
  TEST_ASSERT_EQUAL_FLOAT(US_TO_CYCLES(20), task.metric_mean_acc);
  state.loop_counter = saved_loop_counter;
}

void test_scheduler_queue_insertion_preserves_priority_order() {
  task_t input[5] = {};
  input[0].priority = TASK_PRIORITY_LOW;
  input[1].priority = TASK_PRIORITY_MEDIUM;
  input[2].priority = TASK_PRIORITY_LOW;
  input[3].priority = TASK_PRIORITY_REALTIME;
  input[4].priority = TASK_PRIORITY_MEDIUM;
  task_t *ordered[5] = {};
  scheduler_test_order_tasks(input, 5, ordered);
  TEST_ASSERT_EQUAL_PTR(&input[3], ordered[0]);
  TEST_ASSERT_EQUAL_PTR(&input[1], ordered[1]);
  TEST_ASSERT_EQUAL_PTR(&input[4], ordered[2]);
  TEST_ASSERT_EQUAL_PTR(&input[0], ordered[3]);
  TEST_ASSERT_EQUAL_PTR(&input[2], ordered[4]);
}

static uint32_t observed_period_us;

static void observe_task_period() {
  observed_period_us = task_get_last_period_us(TASK_GPS);
  time_test_advance_us(10);
}

void test_scheduler_publishes_actual_period_before_execution() {
  const task_t saved_task = tasks[TASK_GPS];
  tasks[TASK_GPS] = background_task(10);
  tasks[TASK_GPS].period_cycles = US_TO_CYCLES(5000);
  tasks[TASK_GPS].func = observe_task_period;

  time_test_set_us(0);
  scheduler_test_run(&tasks[TASK_GPS], time_cycles());
  TEST_ASSERT_EQUAL_UINT32(0, observed_period_us);
  time_test_set_us(5125);
  scheduler_test_run(&tasks[TASK_GPS], time_cycles());
  TEST_ASSERT_EQUAL_UINT32(5125, observed_period_us);
  TEST_ASSERT_EQUAL_FLOAT(5000, task_get_period_us(TASK_GPS));

  time_test_set_us(UINT32_MAX - 100);
  scheduler_test_run(&tasks[TASK_GPS], time_cycles());
  time_test_set_us(100);
  scheduler_test_run(&tasks[TASK_GPS], time_cycles());
  TEST_ASSERT_EQUAL_UINT32(201, observed_period_us);
  tasks[TASK_GPS] = saved_task;
}

static bool next_arm_state;
static bool next_in_air;
static uint32_t ground_runs;

static void change_arming_state() {
  flags.arm_state = next_arm_state;
  flags.in_air = next_in_air;
}

static void ground_task() {
  ground_runs++;
}

void test_scheduler_refreshes_mask_after_control() {
  looptime_init();
  const control_flags_t saved_flags = flags;
  task_t input[2] = {background_task(0), background_task(0)};
  input[0].priority = TASK_PRIORITY_REALTIME;
  input[0].func = change_arming_state;
  input[1].mask = TASK_MASK_ON_GROUND;
  input[1].func = ground_task;
  ground_runs = 0;

  flags.arm_state = false;
  flags.in_air = false;
  next_arm_state = true;
  next_in_air = false;
  scheduler_test_run_tasks(input, 2, time_cycles());
  TEST_ASSERT_EQUAL_UINT32(0, ground_runs);

  next_arm_state = false;
  next_in_air = false;
  scheduler_test_run_tasks(input, 2, time_cycles());
  TEST_ASSERT_EQUAL_UINT32(1, ground_runs);

  next_in_air = true;
  scheduler_test_run_tasks(input, 2, time_cycles());
  TEST_ASSERT_EQUAL_UINT32(1, ground_runs);
  flags = saved_flags;
}

void test_scheduler_peak_decay_does_not_overflow() {
  task_t task = background_task(0);
  task.func = timed_task;
  const uint32_t saved_loop_counter = state.loop_counter;
  state.loop_counter = 501;
  task_runtime_us = 18000; // Valid below the 20 ms loop fault threshold.
  scheduler_test_run(&task, time_cycles());

  const uint64_t sample_cycles = US_TO_CYCLES(task_runtime_us);
  const uint32_t expected_peak = (sample_cycles * 511) >> 9;
  TEST_ASSERT_EQUAL_UINT32(expected_peak, task.runtime_peak_ema);
  TEST_ASSERT_EQUAL_UINT32((expected_peak * 9) >> 3, task.runtime_worst);
  TEST_ASSERT_GREATER_THAN_UINT32(US_TO_CYCLES(17000), task.runtime_peak_ema);
  state.loop_counter = saved_loop_counter;
}

void test_scheduler_budget_includes_work_before_dispatch() {
  const uint32_t saved_loop_counter = state.loop_counter;
  const uint32_t start_times[] = {0, UINT32_MAX - 310};
  for (uint32_t start_us : start_times) {
    time_test_set_us(start_us);
    looptime_init();
    state.loop_counter = 0;
    time_test_advance_us(state.looptime_autodetect);
    const uint32_t boundary = looptime_update();
    task_t input[2] = {background_task(70), background_task(10)};
    input[0].priority = TASK_PRIORITY_REALTIME;
    input[0].func = timed_task;
    input[1].func = timed_task;
    task_runtime_us = 70;

    // Model filter/housekeeping work between the loop boundary and dispatch.
    time_test_advance_us(60);
    scheduler_test_run_tasks(input, 2, boundary);
    TEST_ASSERT_TRUE(input[0].has_started);
    TEST_ASSERT_EQUAL_UINT32(boundary, input[0].last_time);
    TEST_ASSERT_FALSE(input[1].has_started);
    TEST_ASSERT_EQUAL_UINT8(1, input[1].runtime_skips);

    // The same 130 us must also be measured by the loop limiter, across wrap.
    // Already past 125 us; no simulated busy-wait required.
    const uint32_t next_boundary = looptime_update();
    TEST_ASSERT_EQUAL_UINT32(130, state.cpu_load);
    TEST_ASSERT_EQUAL_FLOAT(130, state.looptime_us);
    TEST_ASSERT_EQUAL_UINT32(time_cycles(), next_boundary);
  }
  state.loop_counter = saved_loop_counter;
}
