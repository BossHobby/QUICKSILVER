#include "driver/adc.h"

// Native ADC implementation for simulator
static uint16_t adc_raw_values[ADC_CHAN_MAX] = {
    [ADC_CHAN_VREF] = 1489,  // VREFINT_CAL default value
    [ADC_CHAN_TEMP] = 2048,  // Mid-scale temperature
    [ADC_CHAN_VBAT] = 3000,  // ~3.7V battery (scaled)
    [ADC_CHAN_IBAT] = 100,   // ~0.5A current (scaled)
};

static bool converting = false;

static const adc_channel_t adc_channel_defaults[ADC_CHAN_MAX] = {
    [ADC_CHAN_VREF] = {
        .pin = PIN_NONE,
        .dev = ADC_DEVICE1,
        .channel = 0,
    },
    [ADC_CHAN_TEMP] = {
        .pin = PIN_NONE,
        .dev = ADC_DEVICE1,
        .channel = 1,
    },
    [ADC_CHAN_VBAT] = {
        .pin = PIN_A1,
        .dev = ADC_DEVICE1,
        .channel = 2,
    },
    [ADC_CHAN_IBAT] = {
        .pin = PIN_A2,
        .dev = ADC_DEVICE1,
        .channel = 3,
    },
};

void adc_init_hardware() {
  // Initialize channels with default values for simulation
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    adc_pins[i] = adc_channel_defaults[i];
  }
  if (target.vbat == PIN_NONE)
    adc_pins[ADC_CHAN_VBAT].dev = ADC_DEVICE_MAX;
  if (target.ibat == PIN_NONE)
    adc_pins[ADC_CHAN_IBAT].dev = ADC_DEVICE_MAX;
  converting = true;
}

void adc_native_scan() {
  if (!converting)
    return;
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    if (adc_pins[i].dev != ADC_DEVICE_MAX)
      adc_accumulate(static_cast<adc_chan_t>(i), adc_raw_values[i]);
  }
}

float adc_convert_to_temp(float val) {
  // Simple linear conversion for simulation
  // Assuming 25°C at mid-scale
  return 25.0f + (val - 2048.0f) * 0.1f;
}

// For testing, allow setting raw ADC values
uint16_t adc_set_raw_value(adc_chan_t chan, uint16_t value) {
  const uint16_t previous = adc_raw_values[chan];
  adc_raw_values[chan] = value;
  return previous;
}
