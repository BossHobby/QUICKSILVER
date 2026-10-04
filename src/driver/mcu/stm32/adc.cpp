#include "driver/adc.h"

#include "driver/interrupt.h"

typedef struct {
  ADC_TypeDef *adc;
  ADC_Common_TypeDef *common;
} adc_dev_t;

#if defined(STM32G4)
#define ADC_INTERNAL_CHANNEL ADC_DEVICE1
#define LL_ADC_CHANNEL_TEMPSENSOR LL_ADC_CHANNEL_TEMPSENSOR_ADC1
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_640CYCLES_5
#define ADC_CLOCK LL_ADC_CLOCK_SYNC_PCLK_DIV4

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
#define ADC_CLOCK LL_ADC_CLOCK_SYNC_PCLK_DIV4

static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC12_COMMON},
    {.adc = ADC2, .common = ADC12_COMMON},
    {.adc = ADC3, .common = ADC3_COMMON},
};
#else
#define ADC_INTERNAL_CHANNEL ADC_DEVICE1
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_480CYCLES
// The slowest clock stretches each injected sequence to roughly 150-190 us.
#define ADC_CLOCK LL_ADC_CLOCK_SYNC_PCLK_DIV8

static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC},
};
#endif

static float temp_cal_a = 0;
static float temp_cal_b = 0;

#if defined(STM32G4) || defined(STM32H7)
// Hardware-oversampled regular conversions take about 1 ms each; the ISR
// chains them through the configured channels.
static adc_chan_t current_chan;
#else
// ADC1 converts every channel in one injected sequence, interrupting once per
// sequence. Ranks without an external pin convert VREF again.
static constexpr uint32_t injected_ranks[] = {
    LL_ADC_INJ_RANK_1,
    LL_ADC_INJ_RANK_2,
    LL_ADC_INJ_RANK_3,
    LL_ADC_INJ_RANK_4,
};
static_assert(ADC_CHAN_MAX == sizeof(injected_ranks) / sizeof(injected_ranks[0]));
static adc_chan_t injected_chans[ADC_CHAN_MAX];
#endif

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
    if (pin == PIN_NONE)
      break;
    for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
      const gpio_af_t *func = &gpio_pin_afs[i];
      if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_ADC) {
        continue;
      }
#if !defined(STM32G4) && !defined(STM32H7)
      // Every F4/F7 analog pin reaches ADC1, which runs the injected sequence.
      if (ADC_TAG_DEV(func->tag) != ADC_DEVICE1) {
        continue;
      }
#endif

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
    adc_common_init.CommonClock = ADC_CLOCK;
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
  // The injected group converts its ranks in one scan.
  adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
#endif
  LL_ADC_Init(dev->adc, &adc_init);

#if defined(STM32G4) || defined(STM32H7)
  // Average 64 conversions while retaining 12-bit output scaling.
#if defined(STM32G4)
  LL_ADC_SetGainCompensation(dev->adc, 0);
  LL_ADC_ConfigOverSamplingRatioShift(dev->adc, LL_ADC_OVS_RATIO_64, LL_ADC_OVS_SHIFT_RIGHT_6);
#else // STM32H7
  LL_ADC_ConfigOverSamplingRatioShift(dev->adc, 64, LL_ADC_OVS_SHIFT_RIGHT_6);
#endif
  LL_ADC_SetOverSamplingScope(dev->adc, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
#endif

  if (adc_pins[ADC_CHAN_VREF].dev == index) {
#if defined(STM32G4) || defined(STM32H7)
    LL_ADC_SetCommonPathInternalChAdd(dev->common, LL_ADC_PATH_INTERNAL_VREFINT);
#else
    LL_ADC_SetCommonPathInternalCh(dev->common, LL_ADC_PATH_INTERNAL_VREFINT);
#endif
  }

  if (adc_pins[ADC_CHAN_TEMP].dev == index) {
#if defined(STM32G4) || defined(STM32H7)
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
#else
  LL_ADC_INJ_SetTriggerSource(dev->adc, LL_ADC_INJ_TRIG_SOFTWARE);
  // Set the length before the ranks: F4/F7 place ranks relative to it.
  LL_ADC_INJ_SetSequencerLength(dev->adc, LL_ADC_INJ_SEQ_SCAN_ENABLE_4RANKS);
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    const uint32_t channel = adc_pins[injected_chans[i]].channel;
    LL_ADC_SetChannelSamplingTime(dev->adc, channel, ADC_SAMPLINGTIME);
    LL_ADC_INJ_SetSequencerRanks(dev->adc, injected_ranks[i], channel);
  }
#endif

  LL_ADC_Enable(dev->adc);

#if defined(STM32H7) || defined(STM32G4)
  while (LL_ADC_IsActiveFlag_ADRDY(dev->adc) == 0)
    ;
  LL_ADC_EnableIT_EOC(dev->adc);
#else
  LL_ADC_EnableIT_JEOS(dev->adc);
#endif
}

#if defined(STM32G4) || defined(STM32H7)
static void adc_start_conversion(adc_chan_t index) {
  const adc_channel_t *chan = &adc_pins[index];
  const adc_dev_t *dev = &adc_dev[chan->dev];

#ifdef STM32H7
  LL_ADC_SetChannelPreSelection(dev->adc, chan->channel);
#endif

  LL_ADC_SetChannelSamplingTime(dev->adc, chan->channel, ADC_SAMPLINGTIME);
  LL_ADC_REG_SetSequencerRanks(dev->adc, LL_ADC_REG_RANK_1, chan->channel);
  LL_ADC_SetChannelSingleDiff(dev->adc, chan->channel, LL_ADC_SINGLE_ENDED);
  LL_ADC_REG_StartConversion(dev->adc);
}

static void adc_irq_handler() {
  ADC_TypeDef *adc = adc_dev[adc_pins[current_chan].dev].adc;
  if (!LL_ADC_IsActiveFlag_EOC(adc))
    return;
  // Reading DR acknowledges EOC before the next channel starts.
  adc_accumulate(current_chan, LL_ADC_REG_ReadConversionData12(adc));
  do {
    current_chan = static_cast<adc_chan_t>((current_chan + 1) % ADC_CHAN_MAX);
  } while (adc_pins[current_chan].dev == ADC_DEVICE_MAX);
  adc_start_conversion(current_chan);
}
#else
static void adc_irq_handler() {
  if (!LL_ADC_IsActiveFlag_JEOS(ADC1))
    return;
  LL_ADC_ClearFlag_JEOS(ADC1);
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++)
    adc_accumulate(injected_chans[i], LL_ADC_INJ_ReadConversionData12(ADC1, injected_ranks[i]));
  LL_ADC_INJ_StartConversionSWStart(ADC1);
}
#endif

void adc_init_hardware() {
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
  adc_init_pin(ADC_CHAN_VBAT, target.vbat);
  adc_init_pin(ADC_CHAN_IBAT, target.ibat);

#if !defined(STM32G4) && !defined(STM32H7)
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    const adc_chan_t chan = static_cast<adc_chan_t>(i);
    injected_chans[i] = adc_pins[chan].dev == ADC_DEVICE_MAX ? ADC_CHAN_VREF : chan;
  }
#endif

  for (const auto &chan : adc_pins) {
    if (chan.dev != ADC_DEVICE_MAX && !LL_ADC_IsEnabled(adc_dev[chan.dev].adc))
      adc_init_dev(chan.dev);
  }

#if defined(STM32G4)
  interrupt_enable(ADC1_2_IRQn, ADC_PRIORITY);
  interrupt_enable(ADC3_IRQn, ADC_PRIORITY);
  interrupt_enable(ADC4_IRQn, ADC_PRIORITY);
  interrupt_enable(ADC5_IRQn, ADC_PRIORITY);
#else
  interrupt_enable(ADC_IRQn, ADC_PRIORITY);
#ifdef STM32H7
  interrupt_enable(ADC3_IRQn, ADC_PRIORITY);
#endif
#endif

#if defined(STM32G4) || defined(STM32H7)
  current_chan = ADC_CHAN_VREF;
  adc_start_conversion(current_chan);
#else
  LL_ADC_INJ_StartConversionSWStart(ADC1);
#endif
}

#if defined(STM32G4)
extern "C" void ADC1_2_IRQHandler() { adc_irq_handler(); }
extern "C" void ADC3_IRQHandler() { adc_irq_handler(); }
extern "C" void ADC4_IRQHandler() { adc_irq_handler(); }
extern "C" void ADC5_IRQHandler() { adc_irq_handler(); }
#else
extern "C" void ADC_IRQHandler() { adc_irq_handler(); }
#ifdef STM32H7
extern "C" void ADC3_IRQHandler() { adc_irq_handler(); }
#endif
#endif

float adc_convert_to_temp(float val) {
#ifdef STM32H7
  // adc cal is 16bit on h7, shift by 4bit left
  val *= 16;
#endif
  return temp_cal_a * val + temp_cal_b;
}
