#include "drv_adc.h"

#include "debug.h"
#include "defines.h"
#include "drv_gpio.h"
#include "filter.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#include <stm32f4xx_ll_adc.h>
#include <stm32f4xx_ll_bus.h>

uint16_t adc_array[2];
extern profile_t profile;

#ifndef DISABLE_ADC

typedef struct {
  __IO uint16_t word1;
  __IO uint16_t word2;

} adcrefcal;

uint16_t adcref_read(adcrefcal *adcref_address) {
  return adcref_address->word1;
}

float vref_cal;

void adc_init(void) {

  //vref_int only exists on ADC1
  //example case: pc2 additional function ADC123_IN12
  //adc1,2,3 connected to APB2 bus

  //enable APB2 clock for adc1
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  //gpio configuration
  {
    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ANALOG;
    gpio_init.Pull = LL_GPIO_PULL_NO;
    gpio_pin_init(&gpio_init, BATTERYPIN);
  }

  LL_ADC_CommonInitTypeDef adc_common_init;
  adc_common_init.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  LL_ADC_CommonInit(ADC1, &adc_common_init);

  LL_ADC_REG_InitTypeDef adc_reg_init;
  adc_reg_init.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  adc_reg_init.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  adc_reg_init.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &adc_reg_init);

  LL_ADC_InitTypeDef adc_init;
  adc_init.Resolution = LL_ADC_RESOLUTION_12B;
  adc_init.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  adc_init.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &adc_init);

  LL_ADC_SetCommonPathInternalCh(ADC1, LL_ADC_PATH_INTERNAL_VREFINT);
  LL_ADC_Enable(ADC1);

  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);

  LL_ADC_REG_StartConversionSWStart(ADC1);

  // reference is measured a 3.3v, toy boards are powered by 2.8, so a 1.17 multiplier
  // different vccs will translate to a different adc scale factor,
  // so actual vcc is not important as long as the voltage is correct in the end
#ifdef F4
  vref_cal = (3.3f / (float)ADC_REF_VOLTAGE) * (float)(adcref_read((adcrefcal *)0x1FFF7A2A));
#endif
}

#define ADC_CHANNELS 2
// internal adc channels:
// 0 - vbat
// 1 - vref

uint16_t readADC1(int channel) {
  static uint8_t adc_channel_synchronizer; //the next adc channel to run when ready
  static uint8_t adc_last_conversion = 1;  //the last adc channel to run

  uint8_t ready_to_convert = LL_ADC_IsActiveFlag_EOCS(ADC1); //check to see if the last conversion is done

  //will skip if adc is still busy or update the adc_array and request the next conversion
  if (ready_to_convert) {
    // Shove the last adc conversion into the array
    adc_array[adc_last_conversion] = LL_ADC_REG_ReadConversionData12(ADC1);

    // Select the new channel to read
    switch (adc_channel_synchronizer) {
    case 0:
      // Reconfigure the new adc channel
      LL_ADC_SetChannelSamplingTime(ADC1, BATTERY_ADC_CHANNEL, LL_ADC_SAMPLINGTIME_480CYCLES);
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, BATTERY_ADC_CHANNEL);

      // Take note of what channel conversion has just been requested
      adc_last_conversion = 0;
      break;

    case 1:
      // Reconfigure the new adc channel
      LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_480CYCLES);
      LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_VREFINT);

      // Take note of what channel conversion has just been requested
      adc_last_conversion = 1;
      break;
    }

    LL_ADC_REG_StartConversionSWStart(ADC1); // Start the conversion
    adc_channel_synchronizer++;              // Advance the index

    if (adc_channel_synchronizer == ADC_CHANNELS)
      adc_channel_synchronizer = 0; // Start the index over if we reached the end of the list
  }

  return adc_array[channel];
}

#ifndef ADC_SCALEFACTOR
// 12 bit ADC has 4096 steps
//scalefactor = (vref/4096) * (R1 + R2)/R2
#define ADC_SCALEFACTOR ((float)ADC_REF_VOLTAGE / 4096) * ((float)VOLTAGE_DIVIDER_R1 + (float)VOLTAGE_DIVIDER_R2) * (1 / (float)VOLTAGE_DIVIDER_R2)
#endif

float adc_read(int channel) {
  switch (channel) {
  case 0: {
    //vbat
    const float raw_value = readADC1(channel);
#ifdef DEBUG
    lpf(&debug.adcfilt, raw_value, 0.998);
#endif
    return raw_value * ((float)(ADC_SCALEFACTOR * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage)));
  }
  case 1: {
    //reference
    const float raw_value = readADC1(channel);
#ifdef DEBUG
    lpf(&debug.adcreffilt, raw_value, 0.998);
#endif

    if (raw_value == 0) {
      // avoid devision by zero below
      return 0;
    }
    return vref_cal / raw_value;
  }
  default:
    return 0;
  }
}
#else
// // lvc disabled
void adc_init(void) {
}
// dummy function with lvc disabled
float adc_read(int channel) {
  switch (channel) {
  case 0:
    return 4.20f;

  case 1:
    return 1.0f;

  default:
    return 0;
  }
}

#endif
