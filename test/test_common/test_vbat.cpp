#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/adc.h"
#include "driver/time.h"
#include "io/vbat.h"

extern uint8_t adc_active_channels;
extern void adc_set_raw_value(adc_chan_t chan, uint16_t value);

void test_vbat_integrates_current_over_elapsed_time() {
  const auto saved_state = state;
  const auto saved_flags = flags;
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint8_t saved_channels = adc_active_channels;
  uint16_t saved_raw;
  adc_read_raw(ADC_CHAN_IBAT, &saved_raw);

  state = {};
  profile.voltage.lipo_cell_count = 1;
  profile.voltage.ibat_scale = 1000;
  target.ibat = PIN_A2;
  adc_init();
  adc_active_channels = 4;
  adc_set_raw_value(ADC_CHAN_IBAT, 1000);
  time_test_set_us(UINT32_MAX - 1005000);
  vbat_init();
  // Settle the current filter at a constant measured load. The worker runs
  // its measurement pass at a fixed cadence, so iterate at that cadence.
  for (unsigned i = 0; i < 3000; i++) {
    time_test_advance_us(1000);
    TEST_ASSERT_EQUAL(pdMS_TO_TICKS(1), vbat_calc());
  }
  float current_ma;
  adc_read(ADC_CHAN_IBAT, &current_ma);
  TEST_ASSERT_GREATER_THAN_FLOAT(0, current_ma);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, current_ma, state.ibat_sag_filtered);

  state.ibat_drawn = 0;
  uint32_t elapsed_us = 0;
  // Steps below the service period are coalesced by the worker's gate, so
  // every advancing interval must be at least the vbat cadence. Includes a
  // delayed service pass, a repeated timestamp and the clock wrap above.
  // Tolerance stays well above float32 ulp at this magnitude.
  const uint32_t intervals[] = {1000, 1000, 27000, 100000, 0};
  for (uint32_t interval : intervals) {
    time_test_advance_us(interval);
    elapsed_us += interval;
    vbat_calc();
    const float expected_mah = current_ma * (elapsed_us * 1e-6f) / 3600.0f;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected_mah, state.ibat_drawn);
  }

  adc_set_raw_value(ADC_CHAN_IBAT, saved_raw);
  adc_active_channels = saved_channels;
  state = saved_state;
  flags = saved_flags;
  profile = saved_profile;
  target = saved_target;
}
