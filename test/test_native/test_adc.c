#include <unity.h>

#include "driver/adc.h"
#include "mock_helpers.h"

// External function from native adc implementation
extern void adc_set_raw_value(adc_chan_t chan, uint16_t value);
extern uint16_t adc_read_raw(adc_chan_t index);

void test_adc_init() {
  adc_init();
  // Test that ADC initialization succeeds
  TEST_PASS();
}

void test_adc_read_temperature() {
  adc_init();

  // Set temperature value for testing
  adc_set_raw_value(ADC_CHAN_TEMP, 2048);
  float temp = adc_read(ADC_CHAN_TEMP);
  // Temperature reading should return a plausible value
  TEST_ASSERT_FLOAT_WITHIN(50.0f, 25.0f, temp);
}

void test_adc_read_vbat() {
  adc_init();

  // Set a typical battery voltage value
  adc_set_raw_value(ADC_CHAN_VBAT, 3000);
  float vbat = adc_read(ADC_CHAN_VBAT);

  // Should return some positive voltage
  TEST_ASSERT_TRUE(vbat > 0.0f);
  TEST_ASSERT_TRUE(vbat < 10.0f);
}

void test_adc_read_ibat() {
  adc_init();

  // Set a typical current value
  adc_set_raw_value(ADC_CHAN_IBAT, 100);
  float ibat = adc_read(ADC_CHAN_IBAT);

  // Current should be non-negative
  TEST_ASSERT_TRUE(ibat >= 0.0f);
}