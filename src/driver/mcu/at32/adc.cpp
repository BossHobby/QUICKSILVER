#include "driver/adc.h"

#include "driver/interrupt.h"

#define ADC_VREF (3.3f)
#define ADC_TEMP_BASE (1.26f)
#define ADC_TEMP_SLOPE (-0.00423f)

#define ADC_INTERNAL_CHANNEL ADC_DEVICE1

#define ADC_CHANNEL_TEMPSENSOR ADC_CHANNEL_16
#define ADC_CHANNEL_VREFINT ADC_CHANNEL_17

#define ADC_SAMPLINGTIME ADC_SAMPLING_INTERVAL_5CYCLES

// Hardware-oversampled conversions are long; the ISR chains them through the
// configured channels.
static adc_chan_t current_chan;

static adc_type *adc_devs[ADC_DEVICE_MAX] = {
    ADC1,
    ADC2,
    ADC3,
};

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
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
    if (pin == PIN_NONE)
      break;
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
    adc_ordinary_oversample_enable(adc_devs[i], TRUE);

    adc_enable(adc_devs[i], TRUE);
    while (adc_flag_get(adc_devs[i], ADC_RDY_FLAG) == RESET)
      ;

    adc_calibration_init(adc_devs[i]);
    while (adc_calibration_init_status_get(adc_devs[i]))
      ;

    adc_calibration_start(adc_devs[i]);
    while (adc_calibration_status_get(adc_devs[i]))
      ;
    adc_interrupt_enable(adc_devs[i], ADC_OCCE_INT, TRUE);
  }
}

static void adc_start_conversion(adc_chan_t index) {
  const adc_channel_t *chan = &adc_pins[index];
  adc_type *adc = adc_devs[chan->dev];
  // Single conversions have finished before the ISR changes the channel.
  adc_ordinary_channel_set(adc, (adc_channel_select_type)chan->channel, 1, ADC_SAMPLETIME_640_5);
  adc_ordinary_software_trigger_enable(adc, TRUE);
}

void adc_init_hardware() {
  rcc_enable(RCC_ENCODE(ADC1));
  rcc_enable(RCC_ENCODE(ADC2));
  rcc_enable(RCC_ENCODE(ADC3));

  adc_init_pin(ADC_CHAN_VREF, PIN_NONE);
  adc_init_pin(ADC_CHAN_TEMP, PIN_NONE);
  adc_init_pin(ADC_CHAN_VBAT, target.vbat);
  adc_init_pin(ADC_CHAN_IBAT, target.ibat);

  adc_init_dev();
  interrupt_enable(ADC1_2_3_IRQn, ADC_PRIORITY);

  current_chan = ADC_CHAN_VREF;
  adc_start_conversion(current_chan);
}

extern "C" void ADC1_2_3_IRQHandler() {
  adc_type *adc = adc_devs[adc_pins[current_chan].dev];
  if (!adc_flag_get(adc, ADC_OCCE_FLAG))
    return;
  // Reading the data acknowledges completion before the next channel starts.
  adc_accumulate(current_chan, adc_ordinary_conversion_data_get(adc));
  do {
    current_chan = static_cast<adc_chan_t>((current_chan + 1) % ADC_CHAN_MAX);
  } while (adc_pins[current_chan].dev == ADC_DEVICE_MAX);
  adc_start_conversion(current_chan);
}

float adc_convert_to_temp(float val) {
  return (ADC_TEMP_BASE - val * ADC_VREF / 4096) / ADC_TEMP_SLOPE + 25;
}
