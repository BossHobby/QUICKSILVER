#include <unity.h>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "control/control.h"
#include "control/navigation.h"
#ifdef VEHICLE_MULTI
#include "control/multi/navigation.h"
#endif
#include "core/profile.h"
#include "core/tasks.h"
#include "driver/time.h"
#include "driver/timer.h"
#include "util/mutex.h"

extern void flight_test_timing_init();
extern void flight_test_update_loop(uint32_t elapsed_cycles);
#ifdef VEHICLE_MULTI
extern void nav_test_reset();
#endif

static void begin_timing() {
  flight_test_timing_init();
  state.loop_counter = 0;
  state.looptime_autodetect = 125;
  state.uptime = 0;
  state.armtime = 0;
  flags.arm_state = 0;
  TEST_ASSERT_EQUAL_UINT8(0, state.looptime_warning);
}

static void next_loop(uint32_t runtime_us, uint32_t period_us) {
  time_test_advance_us(period_us);
  flight_test_update_loop(US_TO_CYCLES(runtime_us));
}

void test_flight_timing_separates_runtime_and_period_across_wrap() {
  const uint32_t starts[] = {0, UINT32_MAX - 50};
  for (uint32_t start : starts) {
    time_test_set_us(start);
    begin_timing();
    next_loop(70, 125);
    TEST_ASSERT_EQUAL_UINT32(125, state.looptime_us);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000125f, state.looptime);
    TEST_ASSERT_FLOAT_WITHIN(1, 8000, state.looptime_inverse);
    TEST_ASSERT_EQUAL_FLOAT(0, state.armtime);
    flags.arm_state = 1;
    next_loop(80, 250);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000375f, state.uptime);
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, 0.000250f, state.armtime);
    flags.arm_state = 0;
  }
}

void test_flight_rate_fallback_cap_and_no_automatic_speedup() {
  begin_timing();
  state.loop_counter = 200;
  for (unsigned i = 0; i < 199; i++)
    next_loop(140, 250);
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  next_loop(140, 250);
  TEST_ASSERT_EQUAL_FLOAT(250, state.looptime_autodetect);
  for (unsigned i = 0; i < 400; i++)
    next_loop(600, 625);
  TEST_ASSERT_EQUAL_FLOAT(500, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(2, state.looptime_warning);
  for (unsigned i = 0; i < 400; i++)
    next_loop(20, 500);
  TEST_ASSERT_EQUAL_FLOAT(500, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(2, state.looptime_warning);
}

void test_flight_maintenance_reset_excludes_delays_and_restarts_average() {
  begin_timing();
  state.loop_counter = 200;
  for (unsigned i = 0; i < 199; i++)
    next_loop(140, 250);
  flight_reset_runtime();
  next_loop(140, 100000); // A command held the ground configuration mutex.
  TEST_ASSERT_EQUAL_UINT32(100000, state.looptime_us);
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  // The delay is excluded from runtime; the previous overload history is reset.
  for (unsigned i = 0; i < 200; i++)
    next_loop(70, 125);
  TEST_ASSERT_EQUAL_FLOAT(125, state.looptime_autodetect);
  TEST_ASSERT_EQUAL_UINT8(0, state.looptime_warning);
}

#ifdef VEHICLE_MULTI
void test_flight_navigation_cadence_configuration_and_wrap() {
  const auto saved_profile = profile;
  const auto saved_state = state;
  const auto saved_flags = flags;
  const uint32_t starts[] = {0, UINT32_MAX - 5000};
  for (uint32_t start : starts) {
    time_test_set_us(start);
    begin_timing();
    nav_test_reset();
    nav_init();
    profile.serial.gps = SERIAL_PORT_INVALID;
    state.gps_coord = {100, 200};
    state.gps_home = {};
    time_test_advance_us(10000);
    nav_update();
    TEST_ASSERT_EQUAL_INT32(0, state.gps_home.lat);

    // A configured receiver must run even before it has acquired a fix.
    profile.serial.gps = SERIAL_PORT1;
    state.gps_lock = false;
    time_test_advance_us(10000);
    nav_update();
    TEST_ASSERT_EQUAL_INT32(200, state.gps_home.lat);
    state.gps_coord.lat = 300;
    time_test_advance_us(9999);
    nav_update();
    TEST_ASSERT_EQUAL_INT32(200, state.gps_home.lat);
    time_test_advance_us(1);
    nav_update();
    TEST_ASSERT_EQUAL_INT32(300, state.gps_home.lat);

    time_test_advance_us(35000);
    state.gps_coord.lat = 400;
    nav_update();
    TEST_ASSERT_EQUAL_INT32(400, state.gps_home.lat);
    state.gps_coord.lat = 500;
    nav_update(); // No catch-up passes after a delay.
    TEST_ASSERT_EQUAL_INT32(400, state.gps_home.lat);
  }
  nav_test_reset();
  profile = saved_profile;
  state = saved_state;
  flags = saved_flags;
}
#endif

static void timer_test_flight(void *) {
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) _exit(8);
  if (threads[THREAD_BLACKBOX].handle != nullptr) _exit(9);
  thread_start(THREAD_BLACKBOX);
  timer_up_init(TIMER1, PWM_CLOCK_FREQ_HZ / 2000000, 1999); // 1 ms.
  timer_up_start(TIMER1);
  vTaskDelay(1);
  if (!timer_up_pending(TIMER1)) _exit(1);

  // A buffered period change becomes active at the next expiration.
  timer_up_set_period(TIMER1, 5999); // 3 ms at the native 2 MHz timer clock.
  vTaskDelay(1);
  if (!timer_up_pending(TIMER1)) _exit(2);
  vTaskDelay(2);
  if (timer_up_pending(TIMER1)) _exit(3);
  vTaskDelay(1);
  if (!timer_up_pending(TIMER1)) _exit(4);

  // Missed expirations coalesce into one pending update flag.
  vTaskDelay(10);
  if (!timer_up_pending(TIMER1)) _exit(5);
  if (timer_up_pending(TIMER1)) _exit(6);
  _exit(0);
}

static void timer_test_background(void *) {
  for (;;) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void load_test_background(void *) {
  for (;;) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

static void load_test_flight(void *) {
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) _exit(8);
  state.loop_counter = 0;
  state.looptime_autodetect = 125;

  // Starve the idle task: the published load must climb and stay clamped to a
  // percentage. The POSIX port only keeps run-time-stat bookkeeping current at
  // kernel yield points, so yield periodically; the ARM ports preempt exactly
  // and need no such aid. The falling direction (idle share credited while a
  // task blocks in a port-layer wait) is a POSIX-port artifact and is
  // verifiable on hardware only.
  const uint32_t clock0 = rtos_runtime_clock();
  const uint32_t spin_deadline = clock0 + US_TO_CYCLES(300000);
  while ((int32_t)(rtos_runtime_clock() - spin_deadline) < 0) {
    for (unsigned i = 0; i < 100; i++) {
      flight_test_update_loop(0);
    }
    taskYIELD();
  }
  if (state.cpu_load < 50 || state.cpu_load > 100) _exit(1);
  _exit(0);
}

void test_flight_system_load_tracks_idle_share() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = load_test_flight;
    threads[THREAD_BLACKBOX].entry = load_test_background;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(7);
  }
  int status;
  for (unsigned i = 0; i < 2000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("System load test stalled");
}

void test_flight_native_timer_period_and_coalescing() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    state.looptime_autodetect = 125;
    threads[THREAD_FLIGHT].entry = timer_test_flight;
    threads[THREAD_BLACKBOX].entry = timer_test_background;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(7);
  }
  int status;
  for (unsigned i = 0; i < 2000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("Native timer or thread startup stalled");
}

static StaticSemaphore_t context_mutex_storage;
static SemaphoreHandle_t context_mutex;
static volatile unsigned context_worker_passes;

static void context_test_worker(void *) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    mutex_guard_t guard(context_mutex);
    context_worker_passes++;
  }
}

static void context_test_flight(void *) {
  context_mutex = xSemaphoreCreateMutexStatic(&context_mutex_storage);
  flags.arm_state = 0;
  flags.in_air = 0;
  thread_start(THREAD_USB);
  xTaskNotifyGive(threads[THREAD_USB].handle);
  vTaskDelay(2);
  if (context_worker_passes != 1) _exit(1);

  // Let the worker block on configuration ownership, then arm while holding it.
  if (xSemaphoreTake(context_mutex, 0) != pdTRUE) _exit(3);
  {
    mutex_guard_t disabled(context_mutex, false);
  }
  if (xSemaphoreTake(context_mutex, 0) != pdFALSE) _exit(2);
  xTaskNotifyGive(threads[THREAD_USB].handle);
  vTaskDelay(2);
  flags.arm_state = 1;
  threads_update();
  xSemaphoreGive(context_mutex);
  xTaskNotifyGive(threads[THREAD_USB].handle);
  vTaskDelay(2);
  if (context_worker_passes != 1) _exit(4);
  if (xSemaphoreTake(context_mutex, 0) != pdTRUE) _exit(5);
  xSemaphoreGive(context_mutex);

  // Reapplying the same mask must leave the worker suspended, even with a
  // pending notification. Disarming in the air still excludes ground work.
  threads_update();
  flags.arm_state = 0;
  flags.in_air = 1;
  threads_update();
  vTaskDelay(2);
  if (context_worker_passes != 1) _exit(6);

  flags.in_air = 0;
  threads_update();
  vTaskDelay(2);
  if (context_worker_passes < 2) _exit(7);
  if (xSemaphoreTake(context_mutex, 0) != pdTRUE) _exit(8);
  xSemaphoreGive(context_mutex);
  _exit(0);
}

void test_threads_suspend_ground_workers_with_pending_work() {
  const pid_t child = fork();
  TEST_ASSERT_TRUE(child >= 0);
  if (child == 0) {
    threads[THREAD_FLIGHT].entry = context_test_flight;
    threads[THREAD_USB].entry = context_test_worker;
    thread_start(THREAD_FLIGHT);
    vTaskStartScheduler();
    _exit(9);
  }
  int status;
  for (unsigned i = 0; i < 2000; i++) {
    if (waitpid(child, &status, WNOHANG) == child) {
      TEST_ASSERT_TRUE(WIFEXITED(status));
      TEST_ASSERT_EQUAL_INT(0, WEXITSTATUS(status));
      return;
    }
    usleep(1000);
  }
  kill(child, SIGKILL);
  waitpid(child, &status, 0);
  TEST_FAIL_MESSAGE("Thread context transition stalled");
}
