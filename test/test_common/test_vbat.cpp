#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "io/vbat.h"

extern uint16_t adc_set_raw_value(adc_chan_t chan, uint16_t value);

// The ADC converts continuously; model one native scan per elapsed millisecond.
static void vbat_test_advance_ms(unsigned ms) {
  for (unsigned i = 0; i < ms; i++) {
    time_test_advance_us(1000);
    adc_native_scan();
  }
}

static void vbat_test_init() {
  profile.voltage.lipo_cell_count = 1;
  profile.voltage.ibat_scale = 1000;
  target.ibat = PIN_A2;
  target.vbat = PIN_A1;
  adc_init();
  adc_native_scan();
  vbat_init();
}

void test_vbat_runs_on_fixed_period() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  state = {};
  time_test_set_us(UINT32_MAX - 12000);
  vbat_test_init();

  // Early wakes report the remaining time; due passes report a full period.
  for (unsigned ms = 1; ms <= 20; ms++) {
    vbat_test_advance_ms(1);
    const unsigned remaining = 5 - ms % 5;
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(remaining), vbat_calc());
  }

  // A late pass keeps the schedule's phase instead of stretching the period.
  vbat_test_advance_ms(7);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(3), vbat_calc());
  vbat_test_advance_ms(3);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());

  // After a long worker delay the schedule restarts rather than catching up.
  vbat_test_advance_ms(30);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());
  vbat_test_advance_ms(1);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(4), vbat_calc());

  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}

void test_vbat_current_filter_step_response() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint16_t saved_raw = adc_set_raw_value(ADC_CHAN_IBAT, 1000);
  state = {};
  time_test_set_us(UINT32_MAX - 50000);
  vbat_test_init();

  for (unsigned ms = 5; ms <= 100; ms += 5) {
    vbat_test_advance_ms(5);
    vbat_calc();
  }
  // Continuous-time responses at 100 ms, allowing discrete filter error.
  TEST_ASSERT_FLOAT_WITHIN(0.015f, 0.9568f, state.ibat_sag_filtered / state.ibat);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.581f, state.ibat_filtered / state.ibat);

  adc_set_raw_value(ADC_CHAN_IBAT, saved_raw);
  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}

void test_vbat_integrates_current_over_elapsed_time() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint16_t saved_raw = adc_set_raw_value(ADC_CHAN_IBAT, 1000);

  state = {};
  time_test_set_us(UINT32_MAX - 1005000);
  vbat_test_init();
  // Settle the current filter at a constant measured load.
  for (unsigned i = 0; i < 600; i++) {
    vbat_test_advance_ms(5);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(5), vbat_calc());
    if (i == 0)
      TEST_ASSERT_EQUAL_FLOAT(0, state.ibat_drawn);
  }
  const float current_ma = state.ibat;
  TEST_ASSERT_GREATER_THAN_FLOAT(0, current_ma);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, current_ma, state.ibat_sag_filtered);

  state.ibat_drawn = 0;
  uint32_t elapsed_ms = 0;
  // Includes a delayed service pass and the clock wrap above. Tolerance
  // stays well above float32 ulp at this magnitude.
  const uint32_t intervals_ms[] = {5, 5, 27, 100};
  for (uint32_t interval : intervals_ms) {
    vbat_test_advance_ms(interval);
    elapsed_ms += interval;
    vbat_calc();
    const float expected_mah = current_ma * (elapsed_ms * 1e-3f) / 3600.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_mah, state.ibat_drawn);
  }

  // A new load observed after a delay must not be charged to the interval
  // before it was observed. Missing samples keep the previous filtered value.
  float expected_mah = state.ibat_drawn;
  const struct {
    uint16_t raw_current;
    bool available;
  } samples[] = {{2000, true}, {0, false}, {0, true}};
  for (const auto &sample : samples) {
    adc_set_raw_value(ADC_CHAN_IBAT, sample.raw_current);
    const float held_current = state.ibat_sag_filtered;
    expected_mah += held_current * 0.1f / 3600.0f;
    if (sample.available) {
      vbat_test_advance_ms(100);
    } else {
      time_test_advance_us(100000);
    }
    vbat_calc();
    TEST_ASSERT_FLOAT_WITHIN(0.000001f, expected_mah, state.ibat_drawn);
    if (!sample.available)
      TEST_ASSERT_EQUAL_FLOAT(held_current, state.ibat_sag_filtered);
  }

  adc_set_raw_value(ADC_CHAN_IBAT, saved_raw);
  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}
