#include <unity.h>

#include "control/control.h"
#include "core/looptime.h"

extern void looptime_test_auto_detect();

static void sample_runtime(uint32_t runtime_us, uint32_t samples) {
  for (uint32_t i = 0; i < samples; i++) {
    state.cpu_load = runtime_us;
    state.looptime_us = state.looptime_autodetect; // Includes busy-wait padding.
    state.loop_counter++;
    looptime_test_auto_detect();
  }
}

void test_looptime_recovers_after_load_reduces() {
  looptime_init();
  const float minimum = state.looptime_autodetect;
  state.loop_counter = 500;
  sample_runtime(minimum + 20, 200);
  TEST_ASSERT_EQUAL_FLOAT(minimum * 2, state.looptime_autodetect);
  sample_runtime(minimum - 20, 199);
  TEST_ASSERT_EQUAL_FLOAT(minimum * 2, state.looptime_autodetect);
  sample_runtime(minimum - 20, 1);
  TEST_ASSERT_EQUAL_FLOAT(minimum, state.looptime_autodetect);
  sample_runtime(1, 400);
  TEST_ASSERT_EQUAL_FLOAT(minimum, state.looptime_autodetect);
}

void test_looptime_requires_sustained_headroom() {
  looptime_init();
  const float minimum = state.looptime_autodetect;
  state.loop_counter = 500;
  state.looptime_autodetect = minimum * 2;
  sample_runtime(minimum - 20, 199);
  sample_runtime(minimum, 1); // A peak leaves no margin at the faster rate.
  TEST_ASSERT_EQUAL_FLOAT(minimum * 2, state.looptime_autodetect);
  sample_runtime(minimum - 20, 200);
  TEST_ASSERT_EQUAL_FLOAT(minimum, state.looptime_autodetect);
}

void test_looptime_reset_discards_partial_window() {
  looptime_init();
  const float minimum = state.looptime_autodetect;
  state.loop_counter = 500;
  state.looptime_autodetect = minimum * 2;
  sample_runtime(minimum - 20, 199);
  looptime_reset();
  sample_runtime(10000, 1); // Intentional blocking work is excluded.
  sample_runtime(minimum - 20, 199);
  TEST_ASSERT_EQUAL_FLOAT(minimum * 2, state.looptime_autodetect);
  sample_runtime(minimum - 20, 1);
  TEST_ASSERT_EQUAL_FLOAT(minimum, state.looptime_autodetect);
}
