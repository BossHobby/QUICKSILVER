#pragma once

#include <stdint.h>

#include "driver/gpio.h"

typedef enum {
  ADC_DEVICE1,
#if !defined(STM32F411)
  ADC_DEVICE2,
  ADC_DEVICE3,
#endif
#ifdef STM32G473
  ADC_DEVICE4,
  ADC_DEVICE5,
#endif
  ADC_DEVICE_MAX,
} adc_devices_t;

typedef enum {
  ADC_CHAN_VREF,
  ADC_CHAN_TEMP,
  ADC_CHAN_VBAT,
  ADC_CHAN_IBAT,
  ADC_CHAN_MAX,
} adc_chan_t;

typedef struct {
  gpio_pins_t pin;
  adc_devices_t dev;
  uint32_t channel;
} adc_channel_t;

void adc_init();
bool adc_read(adc_chan_t chan, float *val);
uint8_t adc_get_active_channels();
bool adc_read_raw(adc_chan_t chan, uint16_t *val);