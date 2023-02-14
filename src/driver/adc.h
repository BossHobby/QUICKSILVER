#pragma once

typedef enum {
  ADC_CHAN_VREF,
  ADC_CHAN_TEMP,
  ADC_CHAN_VBAT,
  ADC_CHAN_IBAT,
  ADC_CHAN_MAX,
} adc_chan_t;

void adc_init();
float adc_read(adc_chan_t chan);