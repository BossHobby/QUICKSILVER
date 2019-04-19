#include "project.h"
#include "drv_adc.h"
#include "util.h"
#include "config.h"
#include "debug.h"

uint16_t adcarray[2];
extern debug_type debug;


#ifndef DISABLE_LVC


typedef struct {
__IO uint16_t word1;
__IO uint16_t word2;

} adcrefcal;


uint16_t adcref_read(adcrefcal* adcref_address) {
return adcref_address ->word1;
}


float vref_cal;

void adc_init(void)
{	
#ifdef F0	
  ADC_InitTypeDef     ADC_InitStructure;
  
	{
  GPIO_InitTypeDef    GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = BATTERYPIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(BATTERYPORT, &GPIO_InitStructure);
	}

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	{
  DMA_InitTypeDef     DMA_InitStructure;
	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012440;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adcarray;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	}
  
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
 
  ADC_DMACmd(ADC1, ENABLE);  

  ADC_StructInit(&ADC_InitStructure);
  
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
  ADC_Init(ADC1, &ADC_InitStructure); 
	
  ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint , ADC_SampleTime_239_5Cycles); 
 
  ADC_ChannelConfig(ADC1, BATTERY_ADC_CHANNEL , ADC_SampleTime_239_5Cycles); 

  ADC_VrefintCmd(ENABLE);
	
  ADC_GetCalibrationFactor(ADC1);
  
  ADC_Cmd(ADC1, ENABLE);     

  ADC_StartOfConversion(ADC1);
	
  DMA_Cmd(DMA1_Channel1, ENABLE);
#endif

#ifdef F405
//vref_int only exists on ADC1
//pc2 additional function ADC123_IN12
//adc1,2,3 connected to APB2 bus
//dma2 connected on AHB1 bus to APB2 peripherals
//dma2 transmit bufer empty signal is on stream 0, channel 0 / stream 4, channel 0 for adc1
//																			 stream 2, channel 1 / stream 3, channel 1 for adc2
//																			 stream 0, channel 2 / stream 1, channel 2 for adc3
//adc typedef for struct
  ADC_InitTypeDef     ADC_InitStructure;
	
//gpio struct  
	{
  GPIO_InitTypeDef    GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = BATTERYPIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(BATTERYPORT, &GPIO_InitStructure);
	}
	
//enable APB2 clock for adc1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
//enable AHB1 clock for dma1  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 , ENABLE);
	
//dma struct	
	{
  DMA_InitTypeDef     DMA_InitStructure;	
	DMA_InitStructure.DMA_Channel=DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr =  (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcarray;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;    //maybe not needed with fifo disabled
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	}
	
// Enables or disables the ADC DMA request after last transfer 
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
// link it up 
  ADC_DMACmd(ADC1, ENABLE);  
//initialize adc	
  ADC_StructInit(&ADC_InitStructure);
  
//adc struct	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;							//should it be this or none?
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;  									// 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, BATTERY_ADC_CHANNEL , 1 , ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint , 2 , ADC_SampleTime_480Cycles);
	
	ADC_TempSensorVrefintCmd(ENABLE);
  ADC_Cmd(ADC1, ENABLE);     
  ADC_SoftwareStartConv(ADC1);
  DMA_Cmd(DMA2_Stream0, ENABLE);
#endif
 
 // reference is measured a 3.3v, toy boards are powered by 2.8, so a 1.17 multiplier
 // different vccs will translate to a different adc scale factor,
 // so actual vcc is not important as long as the voltage is correct in the end 
 // vref_cal =  1.17857f * (float) ( adcref_read ((adcrefcal *) 0x1FFFF7BA) );
#ifdef F0
vref_cal =  ADC_REF * (float) ( adcref_read ((adcrefcal *) 0x1FFFF7BA) );
#endif
#ifdef F405
vref_cal =  ADC_REF * (float) ( adcref_read ((adcrefcal *) 0x1FFF7A2A) );
#endif
	
}

float adc_read(int channel)
{
	switch(channel)
	{
		case 0:
		#ifdef DEBUG
		lpf(&debug.adcfilt , (float) adcarray[0] , 0.998);
		#endif	
		return (float) adcarray[0] * ((float) (ADC_SCALEFACTOR*(ACTUAL_BATTERY_VOLTAGE/REPORTED_TELEMETRY_VOLTAGE))) ;
		
		case 1:
        #ifdef DEBUG
        lpf(&debug.adcreffilt , (float) adcarray[1] , 0.998);
        #endif	
		return vref_cal / (float) adcarray[1];
		
		default:			
	  return 0;
	}
	
	
}
#else
// // lvc disabled
void adc_init(void)
{
	
}
// dummy function with lvc disabled
float adc_read(int channel)
{
	switch(channel)
	{
		case 0:
		return 4.20f;	
		
		case 1:
		return 1.0f;
		
		default:			
	  return 0;
	}
	
	
}


#endif
