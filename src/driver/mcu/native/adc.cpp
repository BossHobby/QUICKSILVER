#include "driver/adc.h"

#include <stdbool.h>
#include <string.h>

// External declarations for common ADC variables
extern uint16_t adc_array[ADC_CHAN_MAX];
extern adc_channel_t adc_pins[ADC_CHAN_MAX];

// Native ADC implementation for simulator
static uint16_t adc_raw_values[ADC_CHAN_MAX] = {
    [ADC_CHAN_VREF] = 1489,  // VREFINT_CAL default value
    [ADC_CHAN_TEMP] = 2048,  // Mid-scale temperature
    [ADC_CHAN_VBAT] = 3000,  // ~3.7V battery (scaled)
    [ADC_CHAN_IBAT] = 100,   // ~0.5A current (scaled)
};

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

void adc_init() {
  // Initialize channels with default values for simulation
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    adc_pins[i] = adc_channel_defaults[i];
    adc_array[i] = adc_raw_values[i];
  }
}

bool adc_read_raw(adc_chan_t chan, uint16_t *val) {
  if (chan >= ADC_CHAN_MAX || !val) {
    return false;
  }
  
  *val = adc_raw_values[chan];
  return true;
}

float adc_convert_to_temp(uint16_t val) {
  // Simple linear conversion for simulation
  // Assuming 25°C at mid-scale
  return 25.0f + ((float)val - 2048.0f) * 0.1f;
}

// For testing, allow setting raw ADC values
void adc_set_raw_value(adc_chan_t chan, uint16_t value) {
  if (chan < ADC_CHAN_MAX) {
    adc_raw_values[chan] = value;
    adc_array[chan] = value;
  }
}