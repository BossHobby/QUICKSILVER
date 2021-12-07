#pragma once

typedef enum {
  ADC_CHAN_VBAT,
  ADC_CHAN_VREF,
} adc_chan_t;

#define ADC_CHAN_MAX 2

void adc_init();
float adc_read(adc_chan_t chan);