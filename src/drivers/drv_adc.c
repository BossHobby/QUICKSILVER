#include "drv_adc.h"

#include "debug.h"
#include "defines.h"
#include "filter.h"
#include "profile.h"
#include "project.h"
#include "util.h"


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
  ADC_CommonInitTypeDef ADC_CommonInitStructure;		//we may want to use the common init to slow adc down & reduce dma bandwidth taken up on DMA2
  ADC_InitTypeDef ADC_InitStructure;

  //gpio configuration
  {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = BATTERYPIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(BATTERYPORT, &GPIO_InitStructure);
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

  // ADC1 calibration sequence
//  ADC_ResetCalibration(ADC1);
//  while(ADC_GetResetCalibrationStatus(ADC1));
//  ADC_StartCalibration(ADC1);
//  while(ADC_GetCalibrationStatus(ADC1));

  // reference is measured a 3.3v, toy boards are powered by 2.8, so a 1.17 multiplier
  // different vccs will translate to a different adc scale factor,
  // so actual vcc is not important as long as the voltage is correct in the end
  // vref_cal =  1.17857f * (float) ( adcref_read ((adcrefcal *) 0x1FFFF7BA) );
#ifdef F0
  vref_cal = (3.3f / (float)ADC_REF_VOLTAGE) * (float)(adcref_read((adcrefcal *)0x1FFFF7BA));
#endif
#ifdef F4
  vref_cal = (3.3f / (float)ADC_REF_VOLTAGE) * (float)(adcref_read((adcrefcal *)0x1FFF7A2A));
#endif
}

uint16_t readADC1(int channel){
  switch (channel) {  // Select the channel to read
	case 1:
		ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 1, ADC_SampleTime_3Cycles);
		break;
	case 0:
		ADC_RegularChannelConfig(ADC1, BATTERY_ADC_CHANNEL, 1, ADC_SampleTime_3Cycles);
		break;
	}
  // Start the conversion
  ADC_SoftwareStartConv(ADC1);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}

#ifndef ADC_SCALEFACTOR
// 12 bit ADC has 4096 steps
//scalefactor = (vref/4096) * (R1 + R2)/R2
#define ADC_SCALEFACTOR ((float)ADC_REF_VOLTAGE / 4096) * ((float)VOLTAGE_DIVIDER_R1 + (float)VOLTAGE_DIVIDER_R2) * (1 / (float)VOLTAGE_DIVIDER_R2)
#endif

float adc_read(int channel) {
  switch (channel) {
  case 0:	//vbat
#ifdef DEBUG
    lpf(&debug.adcfilt, (float)readADC1(channel), 0.998);
#endif
    return (float)readADC1(channel) * ((float)(ADC_SCALEFACTOR * (profile.voltage.actual_battery_voltage / profile.voltage.reported_telemetry_voltage)));

  case 1:	//reference
#ifdef DEBUG
    lpf(&debug.adcreffilt, (float)readADC1(channel), 0.998);
#endif
    return vref_cal / (float)readADC1(channel);

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
