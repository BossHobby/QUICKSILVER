#include "driver/adc.h"

#define ADC_VREF (3.3f)
#define ADC_TEMP_BASE (1.26f)
#define ADC_TEMP_SLOPE (-0.00423f)

#define ADC_INTERNAL_CHANNEL ADC_DEVICE1

#define ADC_CHANNEL_TEMPSENSOR ADC_CHANNEL_16
#define ADC_CHANNEL_VREFINT ADC_CHANNEL_17

#define ADC_SAMPLINGTIME ADC_SAMPLING_INTERVAL_5CYCLES

extern uint16_t adc_array[ADC_CHAN_MAX];
extern adc_channel_t adc_pins[ADC_CHAN_MAX];

static adc_type *adc_devs[ADC_DEVICEMAX] = {
    ADC1,
    ADC2,
    ADC3,
};

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_array[chan] = 1;
  adc_pins[chan].pin = PIN_NONE;
  adc_pins[chan].dev = ADC_DEVICEMAX;

  switch (chan) {
  case ADC_CHAN_VREF:
    adc_pins[chan].dev = ADC_INTERNAL_CHANNEL;
    adc_pins[chan].channel = ADC_CHANNEL_VREFINT;
    break;

  case ADC_CHAN_TEMP:
    adc_pins[chan].dev = ADC_INTERNAL_CHANNEL;
    adc_pins[chan].channel = ADC_CHANNEL_TEMPSENSOR;
    break;

  default:
    for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
      const gpio_af_t *func = &gpio_pin_afs[i];
      if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_ADC) {
        continue;
      }

      adc_pins[chan].pin = pin;
      adc_pins[chan].dev = ADC_TAG_DEV(func->tag);
      adc_pins[chan].channel = ADC_TAG_CH(func->tag);
      break;
    }
    break;
  }

  if (adc_pins[chan].pin != PIN_NONE) {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_ANALOG;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_pin_init(pin, gpio_init);
  }
}

static void adc_init_dev() {
  adc_common_config_type common_init;
  common_init.combine_mode = ADC_INDEPENDENT_MODE;
  common_init.div = ADC_HCLK_DIV_4;
  common_init.common_dma_mode = ADC_COMMON_DMAMODE_DISABLE;
  common_init.common_dma_request_repeat_state = FALSE;
  common_init.sampling_interval = ADC_SAMPLINGTIME;
  common_init.tempervintrv_state = TRUE;
  common_init.vbat_state = FALSE;
  adc_common_config(&common_init);

  for (uint32_t i = 0; i < ADC_DEVICEMAX; i++) {
    adc_base_config_type base_init;
    base_init.sequence_mode = FALSE;
    base_init.repeat_mode = FALSE;
    base_init.data_align = ADC_RIGHT_ALIGNMENT;
    base_init.ordinary_channel_length = 1;
    adc_base_config(adc_devs[i], &base_init);

    adc_resolution_set(adc_devs[i], ADC_RESOLUTION_12B);
    adc_oversample_ratio_shift_set(adc_devs[i], ADC_OVERSAMPLE_RATIO_64, ADC_OVERSAMPLE_SHIFT_6);

    adc_enable(adc_devs[i], TRUE);
    while (adc_flag_get(adc_devs[i], ADC_RDY_FLAG) == RESET)
      ;

    adc_calibration_init(adc_devs[i]);
    while (adc_calibration_init_status_get(adc_devs[i]))
      ;

    adc_calibration_start(adc_devs[i]);
    while (adc_calibration_status_get(adc_devs[i]))
      ;
  }
}

static void adc_start_conversion(adc_chan_t index) {
  const adc_channel_t *chan = &adc_pins[index];
  adc_type *adc = adc_devs[chan->dev];

  while (adc_flag_get(adc, ADC_RDY_FLAG) == RESET)
    ;

  adc_enable(adc, FALSE);
  adc_ordinary_channel_set(adc, chan->channel, 1, ADC_SAMPLETIME_640_5);
  adc_enable(adc, TRUE);

  while (adc_flag_get(adc, ADC_RDY_FLAG) == RESET)
    ;
  adc_ordinary_software_trigger_enable(adc, TRUE);
}

void adc_init() {
  rcc_enable(RCC_ENCODE(ADC1));
  rcc_enable(RCC_ENCODE(ADC2));
  rcc_enable(RCC_ENCODE(ADC3));

  adc_init_pin(ADC_CHAN_VREF, PIN_NONE);
  adc_init_pin(ADC_CHAN_TEMP, PIN_NONE);
  if (target.vbat != PIN_NONE) {
    adc_init_pin(ADC_CHAN_VBAT, target.vbat);
  }
  if (target.ibat != PIN_NONE) {
    adc_init_pin(ADC_CHAN_IBAT, target.ibat);
  }

  adc_init_dev();
  adc_start_conversion(ADC_CHAN_VREF);
}

uint16_t adc_read_raw(adc_chan_t index) {
  static adc_chan_t last_adc_chan = ADC_CHAN_VREF;

  const adc_channel_t *chan = &adc_pins[last_adc_chan];
  adc_type *adc = adc_devs[chan->dev];

  if (adc_flag_get(adc, ADC_OCCE_FLAG)) {
    adc_array[last_adc_chan] = adc_ordinary_conversion_data_get(adc);

    do {
      last_adc_chan = (last_adc_chan + 1) % ADC_CHAN_MAX;
      // skip through all channels without a dev
    } while (adc_pins[last_adc_chan].dev == ADC_DEVICEMAX);

    adc_start_conversion(last_adc_chan);
  }

  return adc_array[index];
}

float adc_convert_to_temp(float val) {
  return (ADC_TEMP_BASE - val * ADC_VREF / 4096) / ADC_TEMP_SLOPE + 25;
}