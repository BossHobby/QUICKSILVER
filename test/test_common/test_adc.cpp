#include <unity.h>

#include "core/profile.h"
#include "driver/adc.h"

extern uint16_t adc_set_raw_value(adc_chan_t chan, uint16_t value);

void test_adc_init() {
  adc_init();
  TEST_ASSERT_FALSE(adc_update());
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
}

void test_adc_read_temperature() {
  const uint16_t saved = adc_set_raw_value(ADC_CHAN_TEMP, 2048);
  adc_init();
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 25.0f, adc_read(ADC_CHAN_TEMP));
  adc_set_raw_value(ADC_CHAN_TEMP, saved);
}

void test_adc_read_vbat() {
  const uint16_t saved = adc_set_raw_value(ADC_CHAN_VBAT, 3000);
  adc_init();
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  const float vbat = adc_read(ADC_CHAN_VBAT);
  TEST_ASSERT_TRUE(vbat > 0.0f);
  TEST_ASSERT_TRUE(vbat < 10.0f);
  adc_set_raw_value(ADC_CHAN_VBAT, saved);
}

void test_adc_read_ibat() {
  const uint16_t saved = adc_set_raw_value(ADC_CHAN_IBAT, 100);
  adc_init();
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_TRUE(adc_read(ADC_CHAN_IBAT) >= 0.0f);
  adc_set_raw_value(ADC_CHAN_IBAT, saved);
}

void test_adc_current_preserves_fractional_millivolts() {
  const auto saved_profile = profile;
  const auto saved_target = target;
  const uint16_t saved_current = adc_set_raw_value(ADC_CHAN_IBAT, 1);
  const uint16_t saved_vref = adc_set_raw_value(ADC_CHAN_VREF, 1500);
  target.ibat = PIN_A2;
  profile.voltage.ibat_scale = 100;
  adc_init();
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 79.99512f, adc_read(ADC_CHAN_IBAT));

  // Averaging one and two counts keeps the half count.
  adc_native_scan();
  adc_set_raw_value(ADC_CHAN_IBAT, 2);
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 119.99268f, adc_read(ADC_CHAN_IBAT));

  adc_set_raw_value(ADC_CHAN_IBAT, saved_current);
  adc_set_raw_value(ADC_CHAN_VREF, saved_vref);
  profile = saved_profile;
  target = saved_target;
  adc_init();
}

void test_adc_window_averages_every_scan() {
  const auto saved_target = target;
  const uint16_t saved_vref = adc_set_raw_value(ADC_CHAN_VREF, 0);
  target.vbat = PIN_A1;
  target.ibat = PIN_A2;
  adc_init();
  const uint16_t samples[] = {0, 4095, 0, 4095, 1000};
  for (uint16_t raw : samples) {
    adc_set_raw_value(ADC_CHAN_VREF, raw);
    adc_native_scan();
  }
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(1838.0f, adc_read(ADC_CHAN_VREF));

  // A published window is cleared; the next average excludes its samples.
  adc_set_raw_value(ADC_CHAN_VREF, 1500);
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(1500.0f, adc_read(ADC_CHAN_VREF));

  adc_set_raw_value(ADC_CHAN_VREF, saved_vref);
  target = saved_target;
  adc_init();
}

void test_adc_incomplete_window_keeps_accumulating() {
  const auto saved_target = target;
  target.vbat = PIN_A1;
  target.ibat = PIN_A2;
  adc_init();
  adc_accumulate(ADC_CHAN_VREF, 1000);
  adc_accumulate(ADC_CHAN_TEMP, 2048);
  adc_accumulate(ADC_CHAN_VBAT, 3000);
  adc_native_scan();
  TEST_ASSERT_TRUE(adc_update());
  const float vbat = adc_read(ADC_CHAN_VBAT);

  // Chained drivers convert channels one at a time. A window missing a
  // configured channel is neither published nor discarded.
  adc_accumulate(ADC_CHAN_VREF, 1400);
  adc_accumulate(ADC_CHAN_TEMP, 2048);
  adc_accumulate(ADC_CHAN_VBAT, 2000);
  TEST_ASSERT_FALSE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(vbat, adc_read(ADC_CHAN_VBAT));
  adc_accumulate(ADC_CHAN_VREF, 1600);
  adc_accumulate(ADC_CHAN_IBAT, 100);
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(1500.0f, adc_read(ADC_CHAN_VREF));
  TEST_ASSERT_FALSE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(1500.0f, adc_read(ADC_CHAN_VREF));

  target = saved_target;
  adc_init();
}

void test_adc_window_skips_absent_external_channels() {
  const auto saved_target = target;
  target.vbat = PIN_NONE;
  target.ibat = PIN_NONE;
  adc_init();
  adc_accumulate(ADC_CHAN_VREF, 1489);
  adc_accumulate(ADC_CHAN_TEMP, 2048);
  TEST_ASSERT_TRUE(adc_update());
  TEST_ASSERT_EQUAL_FLOAT(4.2f, adc_read(ADC_CHAN_VBAT));
  TEST_ASSERT_EQUAL_FLOAT(0, adc_read(ADC_CHAN_IBAT));
  target = saved_target;
  adc_init();
}
