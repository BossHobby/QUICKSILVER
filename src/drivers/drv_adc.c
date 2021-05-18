#include "drv_adc.h"

#include "debug.h"
#include "defines.h"
#include "drv_gpio.h"
#include "filter.h"
#include "profile.h"
#include "project.h"
#include "util.h"

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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  //adc typedef for struct
  ADC_CommonInitTypeDef ADC_CommonInitStructure; //we may want to use the common init to slow adc down & reduce dma bandwidth taken up on DMA2
  ADC_InitTypeDef ADC_InitStructure;

  //gpio configuration
  {
    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Mode = GPIO_Mode_AN;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_pin_init(&gpio_init, BATTERYPIN);
  }

  // ADC Common Configuration
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  // ADC1 Configuration
  /* Set ADC resolution */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_480Cycles);
  ADC_SoftwareStartConv(ADC1);

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
  static uint8_t adc_channel_synchronizer;                          //the next adc channel to run when ready
  static uint8_t adc_last_conversion = 1;                           //the last adc channel to run
  uint8_t ready_to_convert = ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC); //check to see if the last conversion is done
  if (ready_to_convert) {                                           //will skip if adc is still busy or update the adc_array and request the next conversion
    adc_array[adc_last_conversion] = ADC_GetConversionValue(ADC1);  // Shove the last adc conversion into the array
    switch (adc_channel_synchronizer) {                             // Select the new channel to read
    case 1:
      ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_480Cycles); // Reconfigure the new adc channel
      adc_last_conversion = 1;                                                          // Take note of what channel conversion has just been requested
      break;
    case 0:
      ADC_RegularChannelConfig(ADC1, BATTERY_ADC_CHANNEL, 1, ADC_SampleTime_480Cycles); // Reconfigure the new adc channel
      adc_last_conversion = 0;                                                          // Take note of what channel conversion has just been requested
      break;
    }
    ADC_SoftwareStartConv(ADC1); // Start the conversion
    adc_channel_synchronizer++;  // Advance the index
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
