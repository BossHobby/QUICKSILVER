#include "driver/adc.h"

#include <stdint.h>

#include <FreeRTOS.h>
#include <task.h>

#include "core/profile.h"
#include "core/project.h"

#ifdef USE_ADC

#define VBAT_SCALE (profile.voltage.vbat_scale * (1.f / 10000.f))

adc_channel_t adc_pins[ADC_CHAN_MAX];

// The driver ISR converts continuously and adds every result to the window.
// adc_update() takes the window under a critical section, so the published
// averages of all channels cover the same interval.
static struct {
  uint32_t sum[ADC_CHAN_MAX];
  uint32_t count[ADC_CHAN_MAX];
} window;
static float adc_array[ADC_CHAN_MAX];

void adc_init() {
  window = {};
  for (auto &value : adc_array)
    value = 0;
  adc_init_hardware();
}

void adc_accumulate(adc_chan_t chan, uint16_t raw) {
  window.sum[chan] += raw;
  window.count[chan]++;
}

bool adc_update() {
  uint32_t sum[ADC_CHAN_MAX];
  uint32_t count[ADC_CHAN_MAX];
  bool complete = true;
  taskENTER_CRITICAL();
  for (unsigned i = 0; i < ADC_CHAN_MAX; i++) {
    if (adc_pins[i].dev != ADC_DEVICE_MAX && window.count[i] == 0)
      complete = false;
  }
  // Leave an incomplete window accumulating until every channel converted.
  if (complete) {
    for (unsigned i = 0; i < ADC_CHAN_MAX; i++) {
      sum[i] = window.sum[i];
      count[i] = window.count[i];
    }
    window = {};
  }
  taskEXIT_CRITICAL();
  if (!complete)
    return false;

  for (unsigned i = 0; i < ADC_CHAN_MAX; i++) {
    if (count[i] != 0)
      adc_array[i] = (float)sum[i] / count[i];
  }
  return true;
}

static float adc_to_mv(float raw) {
  // Protect against division by zero
  if (adc_array[ADC_CHAN_VREF] == 0) {
    return 0;
  }
  const float vref_mv = (float)VREFINT_CAL * VREFINT_CAL_VREF / adc_array[ADC_CHAN_VREF];
  return raw * vref_mv / 4095.0f;
}

float adc_read(adc_chan_t chan) {
  const float raw_value = adc_array[chan];
  switch (chan) {
  case ADC_CHAN_TEMP:
    return adc_convert_to_temp(raw_value);

  case ADC_CHAN_VBAT: {
    const float reported_voltage = profile.voltage.reported_telemetry_voltage;
    return (target.vbat == PIN_NONE) ? 4.20f
         : (reported_voltage == 0.0f) ? 0.0f
         : adc_to_mv(raw_value) * VBAT_SCALE * (profile.voltage.actual_battery_voltage / reported_voltage);
  }

  case ADC_CHAN_IBAT: {
    const float ibat_scale = profile.voltage.ibat_scale;
    return (ibat_scale == 0 || target.ibat == PIN_NONE) ? 0
         : adc_to_mv(raw_value) * (10000.0f / ibat_scale);
  }

  default:
    return raw_value;
  }
}

#else
void adc_init() {}

bool adc_update() { return true; }

float adc_read(adc_chan_t chan) {
  return chan == ADC_CHAN_VBAT ? 4.20f : 0.0f;
}

#endif
