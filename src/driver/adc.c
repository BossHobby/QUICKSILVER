#include "driver/adc.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"

#ifdef USE_ADC

#define VBAT_SCALE (profile.voltage.vbat_scale * (1.f / 10000.f))

uint16_t adc_array[ADC_CHAN_MAX];
adc_channel_t adc_pins[ADC_CHAN_MAX];
uint8_t adc_active_channels = 0;

extern float adc_convert_to_temp(uint16_t val);

static uint32_t adc_to_mv(uint16_t raw) {
  const uint32_t vref_mv = (uint32_t)VREFINT_CAL * VREFINT_CAL_VREF / adc_array[ADC_CHAN_VREF];
  return (raw * vref_mv) / 4095;
}

bool adc_read(adc_chan_t chan, float *val) {
  uint16_t raw_value;
  bool updated = adc_read_raw(chan, &raw_value);
  
  if (!updated || val == NULL) {
    return updated;
  }

  switch (chan) {
  case ADC_CHAN_TEMP:
    *val = adc_convert_to_temp(raw_value);
    break;

  case ADC_CHAN_VBAT:
    *val = (target.vbat == PIN_NONE) ? 4.20f 
         : (float)adc_to_mv(raw_value) * VBAT_SCALE * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage);
    break;

  case ADC_CHAN_IBAT:
    *val = (profile.voltage.ibat_scale == 0 || target.ibat == PIN_NONE) ? 0
         : (float)adc_to_mv(raw_value) * (10000.0f / profile.voltage.ibat_scale);
    break;

  default:
    *val = raw_value;
    break;
  }
  
  return updated;
}

uint8_t adc_get_active_channels() {
  return adc_active_channels;
}
#else
void adc_init() {}

bool adc_read_raw(adc_chan_t chan, uint16_t *val) {
  if (val != NULL) {
    *val = 0;
  }
  return true;
}

bool adc_read(adc_chan_t chan, float *val) {
  if (val != NULL) {
    switch (chan) {
    case ADC_CHAN_VBAT:
      *val = 4.20f;
      break;
    default:
      *val = 0;
      break;
    }
  }
  return true;
}

uint8_t adc_get_active_channels() {
  return 1;
}
#endif