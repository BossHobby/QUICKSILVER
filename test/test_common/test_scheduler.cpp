#include <unity.h>

#include "control/control.h"
#include "core/scheduler.h"
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/time.h"

extern bool scheduler_test_task_registered(task_id_t id);
extern bool scheduler_test_should_run(uint32_t start_cycles, uint8_t task_mask, task_t *task);
extern void scheduler_test_run(task_t *task);
extern uint32_t scheduler_test_loop_start();
extern void scheduler_test_update_rate(uint32_t flight_runtime_us);

static task_t background_task(uint32_t runtime_us) {
  task_t task = {};
  task.mask = TASK_MASK_ALWAYS;
  task.priority = TASK_PRIORITY_LOW;
  task.runtime_worst = US_TO_CYCLES(runtime_us);
  return task;
}

static void begin_timing() {
  scheduler_init();
  state.loop_counter = 0;
  state.looptime_autodetect = 125;
  TEST_ASSERT_EQUAL_UINT8(0, state.looptime_warning);
}

static uint32_t next_loop() {
  time_test_advance_us(state.looptime_autodetect);
  return scheduler_test_loop_start();
}

static void request_slowdown() {
  task_t task = background_task(2000);
  const uint32_t boundary = time_cycles();
  for (uint32_t i = 0; i < 32; i++) {
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  }
}

static void short_task() {
  time_test_advance_us(5);
}

void test_scheduler_exhausted_budget_and_wrap() {
  const uint32_t starts[] = {0, UINT32_MAX - 50};
  for (uint32_t start_us : starts) {
    time_test_set_us(start_us);
    begin_timing();
    const uint32_t boundary = next_loop();
    task_t task = background_task(10);
    time_test_advance_us(105);
    TEST_ASSERT_TRUE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
    time_test_advance_us(10);
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
    time_test_advance_us(20);
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
    TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(10), task.runtime_worst);
    task.priority = TASK_PRIORITY_REALTIME;
    TEST_ASSERT_TRUE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  }
}

void test_scheduler_slows_after_starvation_without_forcing_work() {
  begin_timing();
  const task_t saved_sibling = tasks[TASK_OSD];
  tasks[TASK_OSD] = background_task(30);
  const task_t saved = tasks[TASK_GPS];
  auto &task = tasks[TASK_GPS];
  task = background_task(30);
  for (uint32_t i = 0; i < 32; i++) {
    const uint32_t boundary = next_loop();
    time_test_advance_us(110);
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &tasks[TASK_OSD]));
    TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
    TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(30), task.runtime_worst);
  }
  const uint32_t boundary = next_loop();
  TEST_ASSERT_EQUAL_FLOAT(250, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(1, state.looptime_warning);
  TEST_ASSERT_EQUAL_UINT8(0, tasks[TASK_OSD].runtime_skips);
  tasks[TASK_OSD] = saved_sibling;
  TEST_ASSERT_EQUAL_UINT8(0, task.runtime_skips);
  time_test_advance_us(110);
  TEST_ASSERT_TRUE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  task.func = short_task;
  task.runtime_skips = 12;
  scheduler_test_run(&task);
  TEST_ASSERT_EQUAL_UINT8(0, task.runtime_skips);
  tasks[TASK_GPS] = saved;
}

void test_scheduler_masks_periods_and_reset_do_not_trigger_fallback() {
  begin_timing();
  task_t task = background_task(1000);
  const uint32_t boundary = next_loop();
  task.mask = TASK_MASK_ON_GROUND;
  for (uint32_t i = 0; i < 40; i++) {
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  }
  task.mask = TASK_MASK_ALWAYS;
  task.last_time = boundary;
  task.period_cycles = US_TO_CYCLES(1000);
  for (uint32_t i = 0; i < 40; i++) {
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  }
  TEST_ASSERT_EQUAL_UINT8(0, task.runtime_skips);
  next_loop();
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);

  const task_t saved = tasks[TASK_GPS];
  tasks[TASK_GPS].runtime_skips = 31;
  request_slowdown();
  task_reset_runtime();
  TEST_ASSERT_EQUAL_UINT8(0, tasks[TASK_GPS].runtime_skips);
  next_loop();
  next_loop();
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  tasks[TASK_GPS] = saved;
}

void test_scheduler_fallback_cap_and_no_automatic_speedup() {
  begin_timing();
  const task_t saved = tasks[TASK_GPS];
  auto &task = tasks[TASK_GPS];
  task = background_task(1000);
  for (uint32_t i = 0; i < 100; i++) {
    const uint32_t boundary = next_loop();
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  }
  TEST_ASSERT_EQUAL_FLOAT(500, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(2, state.looptime_warning);
  TEST_ASSERT_EQUAL_UINT32(US_TO_CYCLES(1000), task.runtime_worst);
  state.loop_counter = 200;
  for (uint32_t i = 0; i < 500; i++) {
    scheduler_test_update_rate(20);
  }
  TEST_ASSERT_EQUAL_FLOAT(500, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(2, state.looptime_warning);
  state.looptime_autodetect = 1000;
  request_slowdown();
  next_loop();
  TEST_ASSERT_EQUAL_FLOAT(1000, state.looptime_autodetect);
  tasks[TASK_GPS] = saved;
}

void test_scheduler_accounts_for_housekeeping_and_realtime_overload() {
  begin_timing();
  const uint32_t boundary = next_loop();
  // Both housekeeping before dispatch and realtime work consume this budget.
  time_test_advance_us(60);
  time_test_advance_us(70);
  task_t task = background_task(10);
  TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_IN_AIR, &task));
  scheduler_test_loop_start(); // Already past the boundary; no busy-wait.
  TEST_ASSERT_EQUAL_UINT32(130, state.cpu_load);

  state.loop_counter = 200;
  for (uint32_t i = 0; i < 200; i++) {
    time_test_advance_us(140);
    scheduler_test_loop_start();
  }
  TEST_ASSERT_EQUAL_FLOAT(250, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(1, state.looptime_warning);
}

void test_scheduler_ground_only_starvation_does_not_reduce_flight_rate() {
  begin_timing();
  task_t ground = background_task(1000);
  ground.mask = TASK_MASK_ON_GROUND;
  for (uint32_t i = 0; i < 100; i++) {
    const uint32_t boundary = next_loop();
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, TASK_MASK_DEFAULT | TASK_MASK_ON_GROUND, &ground));
  }
  TEST_ASSERT_EQUAL_UINT8(0, ground.runtime_skips);
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(0, state.looptime_warning);
}

static uint32_t task_duration_us;

static void measured_task() {
  time_test_advance_us(task_duration_us);
}

void test_scheduler_ground_work_does_not_indirectly_reduce_flight_rate() {
  time_test_set_us(UINT32_MAX - 200);
  begin_timing();
  uint32_t boundary = next_loop();
  state.loop_counter = 200;
  task_t flight = background_task(0);
  flight.priority = TASK_PRIORITY_REALTIME;
  flight.func = measured_task;
  task_t ground = background_task(0);
  ground.mask = TASK_MASK_ON_GROUND;
  ground.func = measured_task;
  task_t pending = background_task(30);
  const uint8_t mask = TASK_MASK_DEFAULT | TASK_MASK_ON_GROUND;

  for (uint32_t i = 0; i < 200; i++) {
    task_duration_us = 70;
    scheduler_test_run(&flight);
    task_duration_us = 70;
    scheduler_test_run(&ground);
    // Real elapsed time is 140 us, but the flight-only workload leaves 45 us.
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, mask, &pending));
    TEST_ASSERT_EQUAL_UINT8(0, pending.runtime_skips);
    boundary = scheduler_test_loop_start();
    TEST_ASSERT_EQUAL_UINT32(140, state.cpu_load);
    TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  }
  TEST_ASSERT_EQUAL_UINT8(0, state.looptime_warning);

  // Flight-eligible work still requests fallback while disarmed. Ground work
  // cannot hide the fact that 110 us plus the pending 30 us no longer fits.
  for (uint32_t i = 0; i < 32; i++) {
    task_duration_us = 110;
    scheduler_test_run(&flight);
    task_duration_us = 30;
    scheduler_test_run(&ground);
    TEST_ASSERT_FALSE(scheduler_test_should_run(boundary, mask, &pending));
    boundary = scheduler_test_loop_start();
  }
  TEST_ASSERT_EQUAL_FLOAT(250, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(1, state.looptime_warning);
}

void test_scheduler_omits_unconfigured_sensor_tasks() {
  const auto saved_gps = profile.serial.gps;
  const bool saved_baro = state.baro_detected;
  const bool saved_lock = state.gps_lock;
  for (unsigned combination = 0; combination < 4; combination++) {
    const bool gps = combination & 1;
    const bool baro = combination & 2;
    profile.serial.gps = gps ? SERIAL_PORT1 : SERIAL_PORT_INVALID;
    state.baro_detected = baro;
    state.gps_lock = false; // Configured receivers must still acquire a fix.
    scheduler_init();
    TEST_ASSERT_TRUE(scheduler_test_task_registered(TASK_IMU));
    TEST_ASSERT_TRUE(scheduler_test_task_registered(TASK_PID));
    TEST_ASSERT_EQUAL(gps, scheduler_test_task_registered(TASK_GPS));
    TEST_ASSERT_EQUAL(gps && tasks[TASK_NAV].mask != 0, scheduler_test_task_registered(TASK_NAV));
    TEST_ASSERT_EQUAL(baro, scheduler_test_task_registered(TASK_BARO));
    state.gps_lock = true;
    TEST_ASSERT_EQUAL(gps, scheduler_test_task_registered(TASK_GPS));
  }
  profile.serial.gps = saved_gps;
  state.baro_detected = saved_baro;
  state.gps_lock = saved_lock;
  scheduler_init();
}
