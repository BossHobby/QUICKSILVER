#include "driver/adc.h"

#include "core/debug.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/rcc.h"
#include "driver/time.h"
#include "flight/filter.h"
#include "util/util.h"

typedef enum {
  ADC_DEVICE_1,
#ifdef STM32H7
  ADC_DEVICE_2,
  ADC_DEVICE_3,
#endif
  ADC_DEVICE_MAX,
} adc_devs_t;

typedef struct {
  ADC_TypeDef *adc;
  ADC_Common_TypeDef *common;
} adc_dev_t;

typedef struct {
  gpio_pins_t pin;
  adc_devs_t dev;
  uint32_t channel;
} adc_channel_t;

#ifdef STM32H7
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_387CYCLES_5
#define READY_TO_CONVERT(dev) LL_ADC_IsActiveFlag_EOC(dev)
#else
#define ADC_SAMPLINGTIME LL_ADC_SAMPLINGTIME_480CYCLES
#define READY_TO_CONVERT(dev) LL_ADC_IsActiveFlag_EOCS(dev)
#endif

#define VREFINT_CAL (*(VREFINT_CAL_ADDR))
#define VBAT_SCALE ((float)(VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) / (float)(VBAT_DIVIDER_R2) * (1.f / 1000.f))

#ifdef STM32H7
static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC12_COMMON},
    {.adc = ADC2, .common = ADC12_COMMON},
    {.adc = ADC3, .common = ADC3_COMMON},
};

static const adc_channel_t adc_channel_map[] = {
    {.pin = PIN_NONE, .dev = ADC_DEVICE_3, .channel = LL_ADC_CHANNEL_VREFINT},
    {.pin = PIN_NONE, .dev = ADC_DEVICE_3, .channel = LL_ADC_CHANNEL_TEMPSENSOR},

    {.pin = PIN_C0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_10},
    {.pin = PIN_C1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_11},
    {.pin = PIN_C2, .dev = ADC_DEVICE_3, .channel = LL_ADC_CHANNEL_0},
    {.pin = PIN_C3, .dev = ADC_DEVICE_3, .channel = LL_ADC_CHANNEL_1},
    {.pin = PIN_C4, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_4},
    {.pin = PIN_C5, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_8},
    {.pin = PIN_B0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_9},
    {.pin = PIN_B1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_5},
    {.pin = PIN_A0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_16},
    {.pin = PIN_A1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_17},
    {.pin = PIN_A2, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_14},
    {.pin = PIN_A3, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_15},
    {.pin = PIN_A4, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_18},
    {.pin = PIN_A5, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_19},
    {.pin = PIN_A6, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_3},
    {.pin = PIN_A7, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_7},
};
#else
static const adc_dev_t adc_dev[ADC_DEVICE_MAX] = {
    {.adc = ADC1, .common = ADC},
};

static const adc_channel_t adc_channel_map[] = {
    {.pin = PIN_NONE, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_VREFINT},
    {.pin = PIN_NONE, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_TEMPSENSOR},

    {.pin = PIN_C0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_10},
    {.pin = PIN_C1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_11},
    {.pin = PIN_C2, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_12},
    {.pin = PIN_C3, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_13},
    {.pin = PIN_C4, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_14},
    {.pin = PIN_C5, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_15},
    {.pin = PIN_B0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_8},
    {.pin = PIN_B1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_9},
    {.pin = PIN_A0, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_0},
    {.pin = PIN_A1, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_1},
    {.pin = PIN_A2, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_2},
    {.pin = PIN_A3, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_3},
    {.pin = PIN_A4, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_4},
    {.pin = PIN_A5, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_5},
    {.pin = PIN_A6, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_6},
    {.pin = PIN_A7, .dev = ADC_DEVICE_1, .channel = LL_ADC_CHANNEL_7},
};
#endif

#define ADC_CHANNEL_MAP_SIZE (sizeof(adc_channel_map) / sizeof(adc_channel_t))

static uint16_t adc_array[ADC_CHAN_MAX];
static adc_channel_t adc_pins[ADC_CHAN_MAX];

static float temp_cal_a = 0;
static float temp_cal_b = 0;

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_array[chan] = 1;
  adc_pins[chan].pin = PIN_NONE;
  adc_pins[chan].dev = ADC_DEVICE_MAX;

  for (uint32_t i = 0; i < ADC_CHANNEL_MAP_SIZE; i++) {
    const adc_channel_t *adc_chan = &adc_channel_map[i];

    if (adc_chan->pin != pin) {
      continue;
    }
    if (chan == ADC_CHAN_VREF && adc_chan->channel != LL_ADC_CHANNEL_VREFINT) {
      continue;
    }
    if (chan == ADC_CHAN_TEMP && adc_chan->channel != LL_ADC_CHANNEL_TEMPSENSOR) {
      continue;
    }

    adc_pins[chan].pin = pin;
    adc_pins[chan].dev = adc_chan->dev;
    adc_pins[chan].channel = adc_chan->channel;
    break;
  }

  if (adc_pins[chan].pin == PIN_NONE) {
    return;
  }

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ANALOG;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init(&gpio_init, pin);
}

static void adc_init_dev(adc_devs_t index) {
  const adc_dev_t *dev = &adc_dev[index];

  if (!__LL_ADC_IS_ENABLED_ALL_COMMON_INSTANCE(dev->common)) {
    LL_ADC_CommonInitTypeDef adc_common_init;
    LL_ADC_CommonStructInit(&adc_common_init);
    adc_common_init.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
    LL_ADC_CommonInit(dev->common, &adc_common_init);
  }

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
  LL_ADC_REG_Init(dev->adc, &adc_reg_init);

  LL_ADC_InitTypeDef adc_init;
  adc_init.Resolution = LL_ADC_RESOLUTION_12B;
#ifdef STM32H7
  adc_init.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
  adc_init.LowPowerMode = LL_ADC_LP_MODE_NONE;
#else
  adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
#endif
  LL_ADC_Init(dev->adc, &adc_init);

  if (adc_pins[ADC_CHAN_VREF].dev == index) {
    LL_ADC_SetCommonPathInternalCh(dev->common, LL_ADC_PATH_INTERNAL_VREFINT);
  }

  if (adc_pins[ADC_CHAN_TEMP].dev == index) {
    LL_ADC_SetCommonPathInternalCh(dev->common, LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  }

#ifdef STM32H7
  LL_ADC_DisableDeepPowerDown(dev->adc);
  LL_ADC_EnableInternalRegulator(dev->adc);
  time_delay_us(LL_ADC_DELAY_INTERNAL_REGUL_STAB_US);

  while (LL_ADC_IsCalibrationOnGoing(dev->adc) != 0)
    ;

  // should be cycles, but just use us to be sure
  time_delay_us(LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES);
#endif

  LL_ADC_Enable(dev->adc);

#ifdef STM32H7
  while (LL_ADC_IsActiveFlag_ADRDY(dev->adc) == 0)
    ;
#endif
}

static void adc_start_conversion(adc_chan_t index) {
  const adc_channel_t *chan = &adc_pins[index];
  const adc_dev_t *dev = &adc_dev[chan->dev];

#ifdef STM32H7
  LL_ADC_SetChannelPreSelection(dev->adc, chan->channel);
#endif

  LL_ADC_SetChannelSamplingTime(dev->adc, chan->channel, ADC_SAMPLINGTIME);
  LL_ADC_REG_SetSequencerRanks(dev->adc, LL_ADC_REG_RANK_1, chan->channel);

#ifdef STM32H7
  LL_ADC_REG_StartConversion(dev->adc);
#else
  LL_ADC_REG_StartConversionSWStart(dev->adc);
#endif
}

void adc_init() {
#ifdef STM32H7
  rcc_enable(RCC_AHB1_GRP1(ADC12));
  rcc_enable(RCC_AHB4_GRP1(ADC3));
#else
  rcc_enable(RCC_APB2_GRP1(ADC1));
#endif

  temp_cal_a = (float)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / (float)(*TEMPSENSOR_CAL2_ADDR - *TEMPSENSOR_CAL1_ADDR);
  temp_cal_b = (float)TEMPSENSOR_CAL1_TEMP - temp_cal_a * *TEMPSENSOR_CAL1_ADDR;

  adc_init_pin(ADC_CHAN_VREF, PIN_NONE);
  adc_init_pin(ADC_CHAN_TEMP, PIN_NONE);
#ifdef VBAT_PIN
  adc_init_pin(ADC_CHAN_VBAT, VBAT_PIN);
#endif
#ifdef IBAT_PIN
  adc_init_pin(ADC_CHAN_IBAT, IBAT_PIN);
#endif

  for (uint32_t i = 0; i < ADC_DEVICE_MAX; i++) {
    adc_init_dev(i);
  }

  adc_start_conversion(ADC_CHAN_VREF);
}

static uint16_t adc_read_raw(adc_chan_t index) {
  static adc_chan_t last_adc_chan = ADC_CHAN_VREF;

  const adc_channel_t *chan = &adc_pins[last_adc_chan];
  const adc_dev_t *dev = &adc_dev[chan->dev];

  if (READY_TO_CONVERT(dev->adc)) {
    adc_array[last_adc_chan] = LL_ADC_REG_ReadConversionData12(dev->adc);

    do {
      last_adc_chan = (last_adc_chan + 1) % ADC_CHAN_MAX;
      // skip through all channels without a dev
    } while (adc_pins[last_adc_chan].dev == ADC_DEVICE_MAX);

    adc_start_conversion(last_adc_chan);
  }

  return adc_array[index];
}

float adc_convert_to_mv(float value) {
  const float vref = (float)(VREFINT_CAL * VREFINT_CAL_VREF) / (float)adc_read_raw(ADC_CHAN_VREF);
  return value * (vref / 4096.0f);
}

float adc_read(adc_chan_t chan) {
  switch (chan) {
  case ADC_CHAN_TEMP: {
    const float val = adc_read_raw(chan);
    return temp_cal_a * val + temp_cal_b;
  }

  case ADC_CHAN_VBAT:
#ifdef VBAT_PIN
    return adc_convert_to_mv(adc_read_raw(chan)) * VBAT_SCALE * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage);
#else
    return 4.20f;
#endif

  case ADC_CHAN_IBAT:
#ifdef IBAT_PIN
    if (profile.voltage.ibat_scale == 0) {
      return 0;
    }
    return adc_convert_to_mv(adc_read_raw(chan)) * (10000.0f / profile.voltage.ibat_scale);
#else
    return 0;
#endif

  default:
    return adc_read_raw(chan);
  }
}