#include "driver/adc.h"

typedef struct {
  ADC_TypeDef *adc;
  ADC_Common_TypeDef *common;
} adc_dev_t;

#if defined(STM32G4)
#define ADC_INTERNAL_CHANNEL ADC_DEVICE1
#define LL_ADC_CHANNEL_TEMPSENSOR LL_ADC_CHANNEL_TEMPSENSOR_ADC1
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_640CYCLES_5
#define READY_TO_CONVERT(dev) LL_ADC_IsActiveFlag_EOC(dev)

static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC12_COMMON},
    {.adc = ADC2, .common = ADC12_COMMON},
    {.adc = ADC3, .common = ADC345_COMMON},
    {.adc = ADC4, .common = ADC345_COMMON},
    {.adc = ADC5, .common = ADC345_COMMON},
};
#elif defined(STM32H7)
#define ADC_INTERNAL_CHANNEL ADC_DEVICE3
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_387CYCLES_5
#define READY_TO_CONVERT(dev) LL_ADC_IsActiveFlag_EOC(dev)

static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC12_COMMON},
    {.adc = ADC2, .common = ADC12_COMMON},
    {.adc = ADC3, .common = ADC3_COMMON},
};
#else
#define ADC_INTERNAL_CHANNEL ADC_DEVICE1
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_480CYCLES
#define READY_TO_CONVERT(dev) LL_ADC_IsActiveFlag_EOCS(dev)

static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC},
};
#endif

extern uint16_t adc_array[ADC_CHAN_MAX];
extern adc_channel_t adc_pins[ADC_CHAN_MAX];

static float temp_cal_a = 0;
static float temp_cal_b = 0;

static const uint32_t channel_map[] = {
    LL_ADC_CHANNEL_0,
    LL_ADC_CHANNEL_1,
    LL_ADC_CHANNEL_2,
    LL_ADC_CHANNEL_3,
    LL_ADC_CHANNEL_4,
    LL_ADC_CHANNEL_5,
    LL_ADC_CHANNEL_6,
    LL_ADC_CHANNEL_7,
    LL_ADC_CHANNEL_8,
    LL_ADC_CHANNEL_9,
    LL_ADC_CHANNEL_10,
    LL_ADC_CHANNEL_11,
    LL_ADC_CHANNEL_12,
    LL_ADC_CHANNEL_13,
    LL_ADC_CHANNEL_14,
    LL_ADC_CHANNEL_15,
    LL_ADC_CHANNEL_16,
    LL_ADC_CHANNEL_17,
    LL_ADC_CHANNEL_18,
};

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_array[chan] = 1;
  adc_pins[chan].pin = PIN_NONE;
  adc_pins[chan].dev = ADC_DEVICE_MAX;

  switch (chan) {
  case ADC_CHAN_VREF:
    adc_pins[chan].dev = ADC_INTERNAL_CHANNEL;
    adc_pins[chan].channel = LL_ADC_CHANNEL_VREFINT;
    break;

  case ADC_CHAN_TEMP:
    adc_pins[chan].dev = ADC_INTERNAL_CHANNEL;
    adc_pins[chan].channel = LL_ADC_CHANNEL_TEMPSENSOR;
    break;

  default:
    for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
      const gpio_af_t *func = &gpio_pin_afs[i];
      if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_ADC) {
        continue;
      }

      adc_pins[chan].pin = pin;
      adc_pins[chan].dev = ADC_TAG_DEV(func->tag);
      adc_pins[chan].channel = channel_map[ADC_TAG_CH(func->tag)];
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

static void adc_init_dev(uint32_t index) {
  const adc_dev_t *dev = &adc_dev[index];

  if (!__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(dev->common)) {
    LL_ADC_CommonInitTypeDef adc_common_init;
    LL_ADC_CommonStructInit(&adc_common_init);
    adc_common_init.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    LL_ADC_CommonInit(dev->common, &adc_common_init);
  }

  LL_ADC_REG_InitTypeDef adc_reg_init;
  LL_ADC_REG_StructInit(&adc_reg_init);
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
  LL_ADC_REG_Init(dev->adc, &adc_reg_init);

  LL_ADC_InitTypeDef adc_init;
  LL_ADC_StructInit(&adc_init);
  adc_init.Resolution = LL_ADC_RESOLUTION_12B;
#if defined(STM32G4)
  adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  adc_init.LowPowerMode = LL_ADC_LP_MODE_NONE;
#elif defined(STM32H7)
  adc_init.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
  adc_init.LowPowerMode = LL_ADC_LP_MODE_NONE;
#else
  adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
#endif
  LL_ADC_Init(dev->adc, &adc_init);

#if defined(STM32G4) || defined(STM32H7)
  // Configure oversampling for G4/H7 - 64x with shorter sample time for fast conversion
#if defined(STM32G4)
  LL_ADC_SetGainCompensation(dev->adc, 0);
  LL_ADC_ConfigOverSamplingRatioShift(dev->adc, LL_ADC_OVS_RATIO_64, LL_ADC_OVS_SHIFT_RIGHT_6);
#else // STM32H7
  LL_ADC_ConfigOverSamplingRatioShift(dev->adc, 64, LL_ADC_OVS_SHIFT_RIGHT_6);
#endif
  LL_ADC_SetOverSamplingScope(dev->adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
#endif

  if (adc_pins[ADC_CHAN_VREF].dev == index) {
#if defined(STM32G4)
    LL_ADC_SetCommonPathInternalChAdd(dev->common, LL_ADC_PATH_INTERNAL_VREFINT);
#else
    LL_ADC_SetCommonPathInternalCh(dev->common, LL_ADC_PATH_INTERNAL_VREFINT);
#endif
  }

  if (adc_pins[ADC_CHAN_TEMP].dev == index) {
#if defined(STM32G4)
    LL_ADC_SetCommonPathInternalChAdd(dev->common, LL_ADC_PATH_INTERNAL_TEMPSENSOR);
#else
    LL_ADC_SetCommonPathInternalCh(dev->common, LL_ADC_PATH_INTERNAL_TEMPSENSOR);
#endif
  }

#if defined(STM32H7) || defined(STM32G4)
  LL_ADC_DisableDeepPowerDown(dev->adc);
  LL_ADC_EnableInternalRegulator(dev->adc);
  time_delay_us(LL_ADC_DELAY_TEMPSENSOR_STAB_US);

#if defined(STM32G4)
  LL_ADC_StartCalibration(dev->adc, LL_ADC_SINGLE_ENDED);
#endif
  while (LL_ADC_IsCalibrationOnGoing(dev->adc) != 0)
    ;

  // should be cycles, but just use us to be sure
  time_delay_us(LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32);
#endif

  LL_ADC_Enable(dev->adc);

#if defined(STM32H7) || defined(STM32G4)
  while (LL_ADC_IsActiveFlag_ADRDY(dev->adc) == 0)
    ;
#endif
}

static void adc_start_conversion(uint32_t index) {
  const adc_channel_t *chan = &adc_pins[index];
  const adc_dev_t *dev = &adc_dev[chan->dev];

#ifdef STM32H7
  LL_ADC_SetChannelPreSelection(dev->adc, chan->channel);
#endif

  LL_ADC_SetChannelSamplingTime(dev->adc, chan->channel, ADC_SAMPLINGTIME);
  LL_ADC_REG_SetSequencerRanks(dev->adc, LL_ADC_REG_RANK_1, chan->channel);

#if defined(STM32H7) || defined(STM32G4)
  LL_ADC_SetChannelSingleDiff(dev->adc, chan->channel, LL_ADC_SINGLE_ENDED);
  LL_ADC_REG_StartConversion(dev->adc);
#else
  LL_ADC_REG_StartConversionSWStart(dev->adc);
#endif
}

void adc_init() {
#if defined(STM32G4)
  rcc_enable(RCC_AHB2_GRP1(ADC12));
  rcc_enable(RCC_AHB2_GRP1(ADC345));
#elif defined(STM32H7)
  rcc_enable(RCC_AHB1_GRP1(ADC12));
  rcc_enable(RCC_AHB4_GRP1(ADC3));
#else
  rcc_enable(RCC_APB2_GRP1(ADC1));
#endif

  temp_cal_a = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (float)(*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR);
  temp_cal_b = (float)TEMPSENSOR_CAL1_TEMP - temp_cal_a * (float)(*TEMPSENSOR_CAL1_ADDR);

  adc_init_pin(ADC_CHAN_VREF, PIN_NONE);
  adc_init_pin(ADC_CHAN_TEMP, PIN_NONE);
  if (target.vbat != PIN_NONE) {
    adc_init_pin(ADC_CHAN_VBAT, target.vbat);
  }
  if (target.ibat != PIN_NONE) {
    adc_init_pin(ADC_CHAN_IBAT, target.ibat);
  }

  for (uint32_t i = 0; i < ADC_DEVICE_MAX; i++) {
    adc_init_dev(i);
  }

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
  static uint32_t current_chan = ADC_CHAN_VREF;
  static bool updated[ADC_CHAN_MAX] = {false};

  const adc_channel_t *chan = &adc_pins[current_chan];
  const adc_dev_t *dev = &adc_dev[chan->dev];

  if (READY_TO_CONVERT(dev->adc)) {
    // Read conversion data - no oversampling
    adc_array[current_chan] = LL_ADC_REG_ReadConversionData12(dev->adc);
    updated[current_chan] = true;

    // Move to next active channel
    do {
      current_chan = (current_chan + 1) % ADC_CHAN_MAX;
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
#ifdef STM32H7
  // adc cal is 16bit on h7, shift by 4bit left
  val *= 16;
#endif
  return temp_cal_a * val + temp_cal_b;
}