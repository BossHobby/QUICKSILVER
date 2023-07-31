#include "driver/adc.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"

#ifdef USE_ADC

#define VBAT_SCALE (profile.voltage.vbat_scale * (1.f / 10000.f))

uint16_t adc_array[ADC_CHAN_MAX];
adc_channel_t adc_pins[ADC_CHAN_MAX];

extern uint16_t adc_read_raw(adc_chan_t index);
extern float adc_convert_to_temp(float val);

static float adc_convert_to_mv(float value) {
  const float vref = (float)(VREFINT_CAL * VREFINT_CAL_VREF) / (float)adc_read_raw(ADC_CHAN_VREF);
  return value * (vref / 4095.0f);
}

float adc_read(adc_chan_t chan) {
  switch (chan) {
  case ADC_CHAN_TEMP:
    return adc_convert_to_temp(adc_read_raw(chan));

  case ADC_CHAN_VBAT:
    if (target.vbat == PIN_NONE) {
      return 4.20f;
    }
    return adc_convert_to_mv(adc_read_raw(chan)) * VBAT_SCALE * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage);

  case ADC_CHAN_IBAT:
    if (profile.voltage.ibat_scale == 0) {
      return 0;
    }
    if (target.ibat == PIN_NONE) {
      return 0;
    }
    return adc_convert_to_mv(adc_read_raw(chan)) * (10000.0f / profile.voltage.ibat_scale);

  default:
    return adc_read_raw(chan);
  }
}
#else
void adc_init() {}
float adc_read(adc_chan_t chan) {
  switch (chan) {
  case ADC_CHAN_VBAT:
    return 4.20f;
  default:
    return 0;
  }
}
#endif