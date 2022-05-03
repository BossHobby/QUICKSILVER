#include "drv_adc.h"

#include "debug.h"
#include "drv_gpio.h"
#include "flight/filter.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

#ifndef DISABLE_ADC

#define ADC_CHANNEL_MAP_SIZE 16

#ifndef ADC_REF_VOLTAGE
#define ADC_REF_VOLTAGE 3300
#endif

#ifdef STM32H7
#define ADC ADC12_COMMON
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_387CYCLES_5
#else
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_480CYCLES
#endif

// 12 bit ADC has 4096 steps
#define ADC_SCALEFACTOR ((float)(ADC_REF_VOLTAGE) / 4096.f)
#define VBAT_SCALE ((float)(VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) / (float)(VBAT_DIVIDER_R2) * (1.f / 1000000.f))

typedef struct {
  gpio_pins_t pin;
  uint32_t channel;
} adc_channel_t;

extern profile_t profile;

float vref_cal = 0;
uint16_t adc_array[ADC_CHAN_MAX];
adc_channel_t adc_pins[ADC_CHAN_MAX];

static const adc_channel_t adc_channel_map[ADC_CHANNEL_MAP_SIZE] = {
    {.pin = PIN_C0, .channel = LL_ADC_CHANNEL_10},
    {.pin = PIN_C1, .channel = LL_ADC_CHANNEL_11},
    {.pin = PIN_C2, .channel = LL_ADC_CHANNEL_12},
    {.pin = PIN_C3, .channel = LL_ADC_CHANNEL_13},
    {.pin = PIN_C4, .channel = LL_ADC_CHANNEL_14},
    {.pin = PIN_C5, .channel = LL_ADC_CHANNEL_15},
    {.pin = PIN_B0, .channel = LL_ADC_CHANNEL_8},
    {.pin = PIN_B1, .channel = LL_ADC_CHANNEL_9},
    {.pin = PIN_A0, .channel = LL_ADC_CHANNEL_0},
    {.pin = PIN_A1, .channel = LL_ADC_CHANNEL_1},
    {.pin = PIN_A2, .channel = LL_ADC_CHANNEL_2},
    {.pin = PIN_A3, .channel = LL_ADC_CHANNEL_3},
    {.pin = PIN_A4, .channel = LL_ADC_CHANNEL_4},
    {.pin = PIN_A5, .channel = LL_ADC_CHANNEL_5},
    {.pin = PIN_A6, .channel = LL_ADC_CHANNEL_6},
    {.pin = PIN_A7, .channel = LL_ADC_CHANNEL_7},
};

static uint16_t adc_calibration_value() {
#if defined(STM32F4)
  return *((uint16_t *)0x1FFF7A2A);
#elif defined(STM32F745) || defined(STM32F765)
  return *((uint16_t *)0x1FF0F44A);
#elif defined(STM32F722)
  return *((uint16_t *)0x1FF07A2A);
#elif defined(STM32H7)
  return *VREFINT_CAL_ADDR;
#endif
}

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_array[chan] = 0;
  adc_pins[chan].pin = PIN_NONE;

  for (uint32_t i = 0; i < ADC_CHANNEL_MAP_SIZE; i++) {
    if (adc_channel_map[i].pin == pin) {
      adc_pins[chan].pin = pin;
      adc_pins[chan].channel = adc_channel_map[i].channel;
      break;
    }
  }

  if (adc_pins[chan].pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ANALOG;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, pin);
}

void adc_init() {
  // vref_int only exists on ADC1
  // example case: pc2 additional function ADC123_IN12
  // adc1,2,3 connected to APB2 bus

#ifdef STM32H7
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);
#else
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
#endif

#ifdef VBAT_PIN
  adc_init_pin(ADC_CHAN_VBAT, VBAT_PIN);
#endif

#ifdef IBAT_PIN
  adc_init_pin(ADC_CHAN_IBAT, IBAT_PIN);
#endif

  LL_ADC_CommonInitTypeDef adc_common_init;
  adc_common_init.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  LL_ADC_CommonInit(ADC, &adc_common_init);

  LL_ADC_REG_InitTypeDef adc_reg_init;
  adc_reg_init.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  adc_reg_init.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  adc_reg_init.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  adc_reg_init.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
#ifdef STM32H7
  adc_reg_init.DataTransferMode = LL_ADC_REG_DR_TRANSFER;
  adc_reg_init.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
#else
  adc_reg_init.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
#endif
  LL_ADC_REG_Init(ADC1, &adc_reg_init);

  LL_ADC_InitTypeDef adc_init;
  adc_init.Resolution = LL_ADC_RESOLUTION_12B;
#ifdef STM32H7
  adc_init.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
  adc_init.LowPowerMode = LL_ADC_LP_MODE_NONE;
#else
  adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
#endif
  LL_ADC_Init(ADC1, &adc_init);

  LL_ADC_SetCommonPathInternalCh(ADC, LL_ADC_PATH_INTERNAL_VREFINT);
  LL_ADC_Enable(ADC1);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, ADC_SAMPLINGTIME);

#ifdef STM32H7
  LL_ADC_REG_StartConversion(ADC1);
#else
  LL_ADC_REG_StartConversionSWStart(ADC1);
#endif

  // reference is measured a 3.3v, toy boards are powered by 2.8, so a 1.17 multiplier
  // different vccs will translate to a different adc scale factor,
  // so actual vcc is not important as long as the voltage is correct in the end
  vref_cal = 1.0f / ((3.3f / (float)ADC_REF_VOLTAGE) * (float)(adc_calibration_value()));
}

static uint16_t adc_read_raw(adc_chan_t chan) {
// will skip if adc is still busy or update the adc_array and request the next conversion
#ifdef STM32H7
  uint8_t ready_to_convert = LL_ADC_IsActiveFlag_EOC(ADC1);
#else
  uint8_t ready_to_convert = LL_ADC_IsActiveFlag_EOCS(ADC1);
#endif
  if (ready_to_convert) {
    static adc_chan_t last_adc_channel = ADC_CHAN_VREF;

    // Shove the last adc conversion into the array
    adc_array[last_adc_channel] = LL_ADC_REG_ReadConversionData12(ADC1);

    // skip through all channels without a pin
    do {
      last_adc_channel++;
      if (last_adc_channel == ADC_CHAN_MAX) {
        last_adc_channel = 0; // Start the index over if we reached the end of the list
      }
    } while (last_adc_channel != ADC_CHAN_VREF && adc_pins[last_adc_channel].pin == PIN_NONE);

    // Select the new channel to read
    switch (last_adc_channel) {
    case ADC_CHAN_VREF:
      LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, ADC_SAMPLINGTIME);
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
      break;

    default: {
      LL_ADC_SetChannelSamplingTime(ADC1, adc_pins[last_adc_channel].channel, ADC_SAMPLINGTIME);
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, adc_pins[last_adc_channel].channel);
      break;
    }
    }

#ifdef STM32H7
    LL_ADC_REG_StartConversion(ADC1);
#else
    LL_ADC_REG_StartConversionSWStart(ADC1);
#endif
  }
  return adc_array[chan];
}

float adc_read(adc_chan_t chan) {
  switch (chan) {
  case ADC_CHAN_VREF:
    return (float)adc_read_raw(chan) * vref_cal;

  case ADC_CHAN_VBAT:
    return (float)adc_read_raw(chan) * ADC_SCALEFACTOR * VBAT_SCALE * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage);

#ifdef IBAT_PIN
  case ADC_CHAN_IBAT:
    if (profile.voltage.ibat_scale == 0) {
      return 0;
    }
    return (float)adc_read_raw(chan) * ADC_SCALEFACTOR * (10000.0f / profile.voltage.ibat_scale);
#endif

  default:
    return 0;
  }
}

#else

// lvc disabled
void adc_init() {}

// dummy function with lvc disabled
float adc_read(adc_chan_t chan) {
  switch (chan) {
  case 0:
    return 4.20f;

  case 1:
    return 1.0f;

  default:
    return 0;
  }
}

#endif
