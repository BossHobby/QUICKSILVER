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

extern adc_channel_t adc_pins[ADC_CHAN_MAX];

void adc_init();
// Boot, then IO, owns update/read. Update publishes the averages of a window in
// which every configured channel converted at least once and starts the next
// window; otherwise it keeps accumulating. Read scales the last published average.
bool adc_update();
float adc_read(adc_chan_t chan);

// Driver hooks: configure the channels and start converting continuously; the
// completion ISR adds each result to the current window.
void adc_init_hardware();
void adc_accumulate(adc_chan_t chan, uint16_t raw);
float adc_convert_to_temp(float raw);
