/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/



#include "project.h"
#include "drv_hw_i2c.h"
#include "drv_time.h"
#include "defines.h"

#define HW_I2C_ADDRESS I2C_GYRO_ADDRESS 

#if defined (HW_I2C_PINS_PA910) || defined (HW_I2C_PINS_PB67)


// pins for hw i2c , select one only
// select pins PB6 and PB7
// OR select pins PA9 and PA10
//#define HW_I2C_PINS_PB67
//#define HW_I2C_PINS_PA910


// digital filter 0 - 15 ( 0 - off )
#define HW_I2C_DIGITAL_FILTER 15


// 100Khz (slow)
//#define HW_I2C_TIMINGREG 0x10805e89

// 200Khz	
//#define HW_I2C_TIMINGREG 0x2060083e

// 400khz (fast)
//#define HW_I2C_TIMINGREG 0x00901850

// 1000Khz (fast+)
//#define HW_I2C_TIMINGREG 0x00700818

// 1000+Khz (overclock i2c)
//#define HW_I2C_TIMINGREG 0x00400615

// 100Khz (slow) (overclock 64Mhz)
//#define HW_I2C_TIMINGREG 0x10d05880

// 400khz (fast) (overclock 64Mhz)
//#define HW_I2C_TIMINGREG 0x00c0216c

// 1000Khz (fast+) (overclock 64Mhz)
//#define HW_I2C_TIMINGREG 0x00900b22


#ifdef HW_I2C_SPEED_FAST_OC
// 1000+Khz (overclock i2c)
#define HW_I2C_TIMINGREG 0x00400615
#endif

#ifdef HW_I2C_SPEED_FAST2
// 1000Khz (fast+)
#define HW_I2C_TIMINGREG 0x00700818
#endif

#ifdef HW_I2C_SPEED_FAST
// 400khz (fast)
#define HW_I2C_TIMINGREG 0x00901850
#endif

#ifdef HW_I2C_SPEED_SLOW1
// 200Khz	
#define HW_I2C_TIMINGREG 0x2060083e
#endif

#ifdef HW_I2C_SPEED_SLOW2
// 100Khz (slow)
#define HW_I2C_TIMINGREG 0x10805e89
#endif


#ifndef HW_I2C_PINS_PA910
#ifndef HW_I2C_PINS_PB67
#define HW_I2C_PINS_PB67
#endif
#endif

// default if not set
#ifndef HW_I2C_TIMINGREG
// 400khz (fast)
#define HW_I2C_TIMINGREG 0x00901850
#endif

extern int liberror;

void hw_i2c_init( void)
{

GPIO_InitTypeDef gpioinitI2C1;

gpioinitI2C1.GPIO_Mode = GPIO_Mode_AF;
gpioinitI2C1.GPIO_OType = GPIO_OType_OD;
gpioinitI2C1.GPIO_PuPd = GPIO_PuPd_UP;


#ifdef HW_I2C_PINS_PB67
gpioinitI2C1.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
GPIO_Init(GPIOB, &gpioinitI2C1);
#endif
	
#ifdef HW_I2C_PINS_PA910
gpioinitI2C1.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
GPIO_Init(GPIOA, &gpioinitI2C1);
#endif
	


#ifdef HW_I2C_PINS_PB67
GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
#endif

#ifdef HW_I2C_PINS_PA910
GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
#endif


RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE);

#ifdef HW_I2C_PINS_PB67
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
SYSCFG_I2CFastModePlusConfig(SYSCFG_I2CFastModePlus_PB6 , ENABLE);
SYSCFG_I2CFastModePlusConfig(SYSCFG_I2CFastModePlus_PB7 , ENABLE);
#endif

#ifdef HW_I2C_PINS_PA910
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
SYSCFG_I2CFastModePlusConfig(SYSCFG_I2CFastModePlus_PA9 , ENABLE);
SYSCFG_I2CFastModePlusConfig(SYSCFG_I2CFastModePlus_PA10 , ENABLE);
#endif

RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

I2C_InitTypeDef initI2C1;

// I2C_StructInit(&initI2C1);

initI2C1.I2C_Timing = HW_I2C_TIMINGREG;
initI2C1.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
initI2C1.I2C_DigitalFilter = HW_I2C_DIGITAL_FILTER ;
initI2C1.I2C_Mode = I2C_Mode_I2C;
initI2C1.I2C_OwnAddress1 = 0xAB;
initI2C1.I2C_Ack = I2C_Ack_Enable;
initI2C1.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

I2C_Init(I2C1, &initI2C1);
I2C_Cmd(I2C1, ENABLE);  
	
}

//#define I2C_TIMEOUT 50000
//#define I2C_CONDITION i2c_timeout > I2C_TIMEOUT

#define I2C_CONDITION ((i2c_timeout>>13))

int hw_i2c_sendheader( int reg, int bytes)
{

unsigned int i2c_timeout = 0;
//check i2c ready	
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET)
	{
		i2c_timeout++;
		if(I2C_CONDITION)
			{ 
			liberror++;
			return 0;
			}
	}
		
// start transfer	
I2C_TransferHandling(I2C1, HW_I2C_ADDRESS<<1, bytes, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	
//i2c_timeout = 0;
// wait for address to be sent	
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET)
		{
		i2c_timeout++;
		if(I2C_CONDITION)
			{ 
			liberror++;
			return 0;
			}
	}

// send next byte (register location)	
I2C_SendData(I2C1, (uint8_t)reg);
	
//i2c_timeout = 0;
// wait until last data sent
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TXE) == RESET)
	{
	i2c_timeout++;
		if(I2C_CONDITION)
		{ 
		liberror++;
		return 0;
		}
	}
	
return 1;
}



void hw_i2c_writereg( int reg ,int data)
{

unsigned int i2c_timeout = 0;

// send start + writeaddress + register location, common send+receive
hw_i2c_sendheader( reg,2 );
// send register value
I2C_SendData(I2C1, (uint8_t) data);
// wait for finish	
while(I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET)
	{
	i2c_timeout++;
		if(I2C_CONDITION)
		{ 
		liberror++;
		return;
		}
	}

// send stop - end transaction
I2C_GenerateSTOP(I2C1, ENABLE);

	
return;	
}



int hw_i2c_readdata( int reg, int *data, int size )
{

static uint8_t i = 0;
unsigned int i2c_timeout = 0;

	// send start + writeaddress + register location, common send+receive
hw_i2c_sendheader( reg, 1 );

	//send restart + readaddress
I2C_TransferHandling(I2C1, HW_I2C_ADDRESS<<1 , size, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

//wait for data
for(i = 0; i<size; i++)
	{
	i2c_timeout = 0;	
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
		{
		i2c_timeout++;
		if(I2C_CONDITION)
				{
				liberror++;
				return 0;
				}
		}
	data[i] = I2C_ReceiveData(I2C1);
	}

//data received	
return 1;
}



int hw_i2c_readreg( int reg )
{
	int data;
	hw_i2c_readdata( reg, &data, 1 );
	return data;
}

#endif






























