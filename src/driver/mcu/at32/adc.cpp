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

static adc_type *adc_devs[ADC_DEVICE_MAX] = {
    ADC1,
    ADC2,
    ADC3,
};

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_array[chan] = 1;
  adc_pins[chan].pin = PIN_NONE;
  adc_pins[chan].dev = ADC_DEVICE_MAX;

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
    gpio_init.output = GPIO_OPENDRAIN;
    gpio_init.drive = GPIO_DRIVE_NORMAL;
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

  for (uint32_t i = 0; i < ADC_DEVICE_MAX; i++) {
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
  adc_ordinary_channel_set(adc, (adc_channel_select_type)chan->channel, 1, ADC_SAMPLETIME_640_5);
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

  // Count active channels
  extern uint8_t adc_active_channels;
  adc_active_channels = 0;
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    if (adc_pins[i].dev != ADC_DEVICE_MAX) {
      adc_active_channels++;
    }
  }
}

bool adc_read_raw(adc_chan_t index, uint16_t *val) {
  static adc_chan_t current_chan = ADC_CHAN_VREF;
  static bool updated[ADC_CHAN_MAX] = {false};

  const adc_channel_t *chan = &adc_pins[current_chan];
  adc_type *adc = adc_devs[chan->dev];

  if (adc_flag_get(adc, ADC_OCCE_FLAG)) {
    // Store conversion result
    adc_array[current_chan] = adc_ordinary_conversion_data_get(adc);
    updated[current_chan] = true;

    // Move to next active channel
    do {
      current_chan = (adc_chan_t)((current_chan + 1) % ADC_CHAN_MAX);
    } while (adc_pins[current_chan].dev == ADC_DEVICE_MAX);

    adc_start_conversion(current_chan);
  }

  // Return requested channel data
  if (updated[index]) {
    if (val)
      *val = adc_array[index];
    updated[index] = false;
    return true;
  }
  return false;
}

float adc_convert_to_temp(uint16_t val) {
  return (ADC_TEMP_BASE - val * ADC_VREF / 4096) / ADC_TEMP_SLOPE + 25;
}