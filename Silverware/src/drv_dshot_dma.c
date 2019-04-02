
// Dshot driver for H101_dual firmware. Written by Markus Gritsch.
// Modified by JazzMac to support DMA transfer

	// DShot timer/DMA init
	// TIM1_UP  DMA_CH5: set all output to HIGH		at TIM1 update
	// TIM1_CH1 DMA_CH2: reset output if data=0		at T0H timing
	// TIM1_CH4 DMA_CH4: reset all output					at T1H timing

// this DMA driver is done with the reference to http://www.cnblogs.com/shangdawei/p/4762035.html

// No throttle jitter, no min/max calibration, just pure digital goodness :)
// Dshot150 would be fast enough for up to 8 kHz main loop frequency. But
// since this implementation does simple bit banging, Dshot150 takes a lot of
// our 1 ms main loop time. Dshot300 takes less time for bit banging, which
// leaves more idle time. Implementing the driver using DMA (like Betaflight
// does) is left as an excercise for the reader ;)

// The ESC signal must be taken before the FET, i.e. non-inverted. The
// signal after the FET with a pull-up resistor is not good enough.
// Bit-bang timing tested only with Keil compiler.

// Dshot capable ESCs required. Consider removing the input filter cap,
// especially if you get drop outs. Tested on "Racerstar MS Series 15A ESC
// BLHeLi_S OPTO 2-4S" ESCs (rebranded ZTW Polaris) with A_H_20_REV16_43.HEX
// and removed filter cap.

// USE AT YOUR OWN RISK. ALWAYS REMOVE PROPS WHEN TESTING.

// Enable this for 3D. The 'Motor Direction' setting in BLHeliSuite must
// be set to 'Bidirectional' (or 'Bidirectional Rev.') accordingly:
//#define BIDIRECTIONAL

// Select Dshot150 or Dshot300. Dshot150 consumes quite some main loop time.
// DShot300 may require removing the input filter cap on the ESC:

#define DSHOT600
//#define DSHOT300
//#define DSHOT150



#ifdef DSHOT150
	#define DSHOT_BIT_TIME 		((SYS_CLOCK_FREQ_HZ/1000/150)-1)
	#define DSHOT_T0H_TIME 		(DSHOT_BIT_TIME*0.30 + 0.05 )
  #define DSHOT_T1H_TIME 		(DSHOT_BIT_TIME*0.60 + 0.05 )
#endif
#ifdef DSHOT300
	#define DSHOT_BIT_TIME 		((SYS_CLOCK_FREQ_HZ/1000/300)-1)
	#define DSHOT_T0H_TIME 		(DSHOT_BIT_TIME*0.30 + 0.05 )
  #define DSHOT_T1H_TIME 		(DSHOT_BIT_TIME*0.60 + 0.05 )
#endif
#ifdef DSHOT600
	#define DSHOT_BIT_TIME 		((SYS_CLOCK_FREQ_HZ/1000/600)-1)
	#define DSHOT_T0H_TIME 		(DSHOT_BIT_TIME*0.30 + 0.05 )
  #define DSHOT_T1H_TIME 		(DSHOT_BIT_TIME*0.60 + 0.05 )
#endif

// IDLE_OFFSET is added to the throttle. Adjust its value so that the motors
// still spin at minimum throttle.
#define IDLE_OFFSET 40

// READ THIS:

// Test the whole throttle range before flight!
// If motors don't stop, turn off TX and wait 2 seconds

// Dshot600 is sensitive to capacitance from wires,
// but should be insensitive to gpio combinations

// Dshot300 is most sensitive to mixes of gpioA
// it has fastest send time in this implementation

// Dshot150 is pretty insensitive to pin mixes and wire capacitance

#include "project.h"

#include "config.h"
#include "defines.h"
#include "drv_pwm.h"
#include "drv_time.h"
#include "hardware.h"
#include "util.h"
#include "drv_dshot.h"
#include "config.h"

#ifdef USE_DSHOT_DMA_DRIVER

#ifdef THREE_D_THROTTLE
#error "Not tested with THREE_D_THROTTLE config option"
#endif

#ifdef INVERTED_ENABLE
#ifndef BIDIRECTIONAL
#error INVERTED_ENABLE is on but not BIDIRECTIONAL in dshot driver
#endif
#endif

#if defined(MOTOR0_PIN_PB0) || defined(MOTOR0_PIN_PB1) || defined(MOTOR1_PIN_PB0) || defined(MOTOR1_PIN_PB1) ||defined(MOTOR2_PIN_PB0) || defined(MOTOR2_PIN_PB1) || defined(MOTOR3_PIN_PB0) || defined(MOTOR3_PIN_PB1)
	#define DSHOT_DMA_PHASE	2												// motor pins at both portA and portB
#else
	#define DSHOT_DMA_PHASE	1												// motor pins all at portA
#endif

extern int failsafe;
extern int onground;

int pwmdir = 0;
static unsigned long pwm_failsafe_time = 1;

volatile int dshot_dma_phase = 0;									// 1:portA  2:portB	 0:idle
volatile uint16_t dshot_packet[4];								// 16bits dshot data for 4 motors

volatile uint16_t motor_data_portA[ 16 ] = { 0 };	// DMA buffer: reset output when bit data=0 at TOH timing
volatile uint16_t motor_data_portB[ 16 ] = { 0 };	//

volatile uint16_t dshot_portA[1] = { 0 };					// sum of all motor pins at portA
volatile uint16_t dshot_portB[1] = { 0 };					// sum of all motor pins at portB

typedef enum { false, true } bool;
void make_packet( uint8_t number, uint16_t value, bool telemetry );

#ifndef FORWARD
#define FORWARD 0
#define REVERSE 1
#endif

// normal output mode
#define gpioset( port , pin) port->BSRR = pin
#define gpioreset( port , pin) port->BRR = pin

//inverted output
//#define gpioset( port , pin) port->BRR = pin
//#define gpioreset( port , pin) port->BSRR = pin

void pwm_init()
{
 GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_0 ;
	GPIO_Init( DSHOT_PORT_0, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_1 ;
	GPIO_Init( DSHOT_PORT_1, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_2 ;
	GPIO_Init( DSHOT_PORT_2, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = DSHOT_PIN_3 ;
	GPIO_Init( DSHOT_PORT_3, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	if( DSHOT_PORT_0 == GPIOA )	*dshot_portA |= DSHOT_PIN_0;
	else												*dshot_portB |= DSHOT_PIN_0;
	if( DSHOT_PORT_1 == GPIOA )	*dshot_portA |= DSHOT_PIN_1;
	else												*dshot_portB |= DSHOT_PIN_1;
	if( DSHOT_PORT_2 == GPIOA )	*dshot_portA |= DSHOT_PIN_2;
	else												*dshot_portB |= DSHOT_PIN_2;
	if( DSHOT_PORT_3 == GPIOA )	*dshot_portA |= DSHOT_PIN_3;
	else												*dshot_portB |= DSHOT_PIN_3;

// DShot timer/DMA init
	// TIM1_UP  DMA_CH5: set all output to HIGH		at TIM1 update
	// TIM1_CH1 DMA_CH2: reset output if data=0		at T0H timing
	// TIM1_CH4 DMA_CH4: reset all output					at T1H timing

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
	// TIM1 Periph clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 						DSHOT_BIT_TIME;
	TIM_TimeBaseStructure.TIM_Prescaler = 				0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 		0;
	TIM_TimeBaseStructure.TIM_CounterMode = 			TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM1, DISABLE);

	/* Timing Mode configuration: Channel 1 */
	TIM_OCInitStructure.TIM_OCMode = 							TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = 				TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 							DSHOT_T0H_TIME;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

	/* Timing Mode configuration: Channel 4 */
	TIM_OCInitStructure.TIM_OCMode = 							TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = 				TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 							DSHOT_T1H_TIME;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);

	DMA_InitTypeDef DMA_InitStructure;

	DMA_StructInit(&DMA_InitStructure);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 Channe5 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel5);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&GPIOA->BSRR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)dshot_portA;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 						16;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	/* DMA1 Channel2 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel2);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&GPIOA->BRR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)motor_data_portA;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 						16;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* DMA1 Channel4 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = 		(uint32_t)&GPIOA->BRR;
	DMA_InitStructure.DMA_MemoryBaseAddr = 				(uint32_t)dshot_portA;
	DMA_InitStructure.DMA_DIR = 									DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = 						16;
	DMA_InitStructure.DMA_PeripheralInc = 				DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = 						DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = 		DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = 				DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = 									DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = 							DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = 									DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	/* configure DMA1 Channel4 interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = 					DMA1_Channel4_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 	(uint8_t)DMA_Priority_High;
	NVIC_InitStructure.NVIC_IRQChannelCmd = 			ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* enable DMA1 Channel4 transfer complete interrupt */
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

	// set failsafetime so signal is off at start
	pwm_failsafe_time = gettime() - 100000;
	pwmdir = FORWARD;
}

void dshot_dma_portA()
{
	DMA1_Channel5->CPAR = (uint32_t)&GPIOA->BSRR;
	DMA1_Channel5->CMAR = (uint32_t)dshot_portA;
	DMA1_Channel2->CPAR = (uint32_t)&GPIOA->BRR;
	DMA1_Channel2->CMAR = (uint32_t)motor_data_portA;
	DMA1_Channel4->CPAR = (uint32_t)&GPIOA->BRR;
	DMA1_Channel4->CMAR = (uint32_t)dshot_portA;

	DMA_ClearFlag( DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5 );

	DMA1_Channel5->CNDTR = 16;
	DMA1_Channel2->CNDTR = 16;
	DMA1_Channel4->CNDTR = 16;

	TIM1->SR = 0;

	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);

	TIM_SetCounter( TIM1, DSHOT_BIT_TIME );
	TIM_Cmd( TIM1, ENABLE );
}

void dshot_dma_portB()
{
	DMA1_Channel5->CPAR = (uint32_t)&GPIOB->BSRR;
	DMA1_Channel5->CMAR = (uint32_t)dshot_portB;
	DMA1_Channel2->CPAR = (uint32_t)&GPIOB->BRR;
	DMA1_Channel2->CMAR = (uint32_t)motor_data_portB;
	DMA1_Channel4->CPAR = (uint32_t)&GPIOB->BRR;
	DMA1_Channel4->CMAR = (uint32_t)dshot_portB;

	DMA_ClearFlag( DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5 );

	DMA1_Channel5->CNDTR = 16;
	DMA1_Channel2->CNDTR = 16;
	DMA1_Channel4->CNDTR = 16;

	TIM1->SR = 0;

	DMA_Cmd(DMA1_Channel2, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);

	TIM_SetCounter( TIM1, DSHOT_BIT_TIME );
	TIM_Cmd( TIM1, ENABLE );
}

// make dshot packet
void make_packet( uint8_t number, uint16_t value, bool telemetry )
{
	uint16_t packet = ( value << 1 ) | ( telemetry ? 1 : 0 ); // Here goes telemetry bit
	// compute checksum
	uint16_t csum = 0;
	uint16_t csum_data = packet;

	for ( uint8_t i = 0; i < 3; ++i ) {
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}

	csum &= 0xf;
	// append checksum
	dshot_packet[ number ] = ( packet << 4 ) | csum;
}

// make dshot dma packet, then fire
void dshot_dma_start()
{
	uint32_t	time=gettime();
	while( dshot_dma_phase != 0 && (gettime()-time) < LOOPTIME ) { } 	// wait maximum a LOOPTIME for dshot dma to complete
	if( dshot_dma_phase != 0 ) return;																// skip this dshot command

#if	defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
	/// terminate current RGB transfer
	extern int	rgb_dma_phase;

	time=gettime();
	while( rgb_dma_phase ==1 && (gettime()-time) < LOOPTIME ) { } 		// wait maximum a LOOPTIME for RGB dma to complete

	if( rgb_dma_phase ==1 ) {																					// terminate current RGB dma transfer, proceed dshot
		rgb_dma_phase =0;
		DMA_Cmd(DMA1_Channel5, DISABLE);
		DMA_Cmd(DMA1_Channel2, DISABLE);
		DMA_Cmd(DMA1_Channel4, DISABLE);

		TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, DISABLE);
		TIM_Cmd( TIM1, DISABLE );
		extern void failloop();
		failloop(9);
	}
#endif

		// generate dshot dma packet
	for ( uint8_t i = 0; i < 16; i++ ) {
		motor_data_portA[ i ] = 0;
		motor_data_portB[ i ] = 0;

	  if ( !( dshot_packet[0] & 0x8000 ) ) {
			if( DSHOT_PORT_0 == GPIOA )	motor_data_portA[ i ] |= DSHOT_PIN_0;
			else 												motor_data_portB[ i ] |= DSHOT_PIN_0; }
	  if ( !( dshot_packet[1] & 0x8000 ) ) {
			if( DSHOT_PORT_1 == GPIOA )	motor_data_portA[ i ] |= DSHOT_PIN_1;
			else 												motor_data_portB[ i ] |= DSHOT_PIN_1; }
	  if ( !( dshot_packet[2] & 0x8000 ) ) {
			if( DSHOT_PORT_2 == GPIOA )	motor_data_portA[ i ] |= DSHOT_PIN_2;
			else 												motor_data_portB[ i ] |= DSHOT_PIN_2; }
	  if ( !( dshot_packet[3] & 0x8000 ) ) {
			if( DSHOT_PORT_3 == GPIOA )	motor_data_portA[ i ] |= DSHOT_PIN_3;
			else 												motor_data_portB[ i ] |= DSHOT_PIN_3; }

	  dshot_packet[0] <<= 1;
	  dshot_packet[1] <<= 1;
	  dshot_packet[2] <<= 1;
	  dshot_packet[3] <<= 1;
	}

	dshot_dma_phase = DSHOT_DMA_PHASE;

	TIM1->ARR 	= DSHOT_BIT_TIME;
	TIM1->CCR1 	= DSHOT_T0H_TIME;
	TIM1->CCR4 	= DSHOT_T1H_TIME;

	DMA1_Channel2->CCR |= DMA_MemoryDataSize_HalfWord|DMA_PeripheralDataSize_HalfWord;	// switch from byte to halfword

	dshot_dma_portA();
}

void pwm_set( uint8_t number, float pwm )
{
    // if ( number > 3 ) failloop(5);
    if ( number > 3 ) return;

	if ( pwm < 0.0f ) {
		pwm = 0.0;
	}
	if ( pwm > 0.999f ) {
		pwm = 0.999;
	}

	uint16_t value = 0;

#ifdef BIDIRECTIONAL

	if ( pwmdir == FORWARD ) {
		// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET .. 1047
		value = 48 + IDLE_OFFSET + (uint16_t)( pwm * ( 1000 - IDLE_OFFSET ) );
	} else if ( pwmdir == REVERSE ) {
		// maps 0.0 .. 0.999 to 1048 + IDLE_OFFSET .. 2047
		value = 1048 + IDLE_OFFSET + (uint16_t)( pwm * ( 1000 - IDLE_OFFSET ) );
	}

#else

	// maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
	value = 48 + IDLE_OFFSET * 2 + (uint16_t)( pwm * ( 2001 - IDLE_OFFSET * 2 ) );

#endif

	if ( onground ) {
		value = 0; // stop the motors
	}

	if ( failsafe ) {
		if ( ! pwm_failsafe_time ) {
			pwm_failsafe_time = gettime();
		} else {
			// 1s after failsafe we turn off the signal for safety
            // this means the escs won't rearm correctly after 2 secs of signal lost
            // usually the quad should be gone by then
			if ( gettime() - pwm_failsafe_time > 4000000 ) {
				value = 0;
								/*
                gpioreset( DSHOT_PORT_0, DSHOT_PIN_0 );
                gpioreset( DSHOT_PORT_1, DSHOT_PIN_1 );
                gpioreset( DSHOT_PORT_2, DSHOT_PIN_2 );
                gpioreset( DSHOT_PORT_3, DSHOT_PIN_3 );
								*/
                //////
                return;
			}
		}
	} else {
		pwm_failsafe_time = 0;
	}

	make_packet( number, value, false );

	if ( number == 3 ) {
			dshot_dma_start();
	}
}

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#ifndef MOTOR_BEEPS_TIMEOUT
#define MOTOR_BEEPS_TIMEOUT 1e6
#endif

void motorbeep()
{
	static unsigned long motor_beep_time = 0;
	if ( failsafe ) {
		unsigned long time = gettime();
		if ( motor_beep_time == 0 ) {
			motor_beep_time = time;
		}
		const unsigned long delta_time = time - motor_beep_time;
		if ( delta_time > MOTOR_BEEPS_TIMEOUT ) {
			uint8_t beep_command = 0;
			if ( delta_time % 2000000 < 250000 ) {
				beep_command = DSHOT_CMD_BEEP1;
			} else if ( delta_time % 2000000 < 500000 ) {
				beep_command = DSHOT_CMD_BEEP3;
			} else if ( delta_time % 2000000 < 750000 ) {
				beep_command = DSHOT_CMD_BEEP2;
			} else if ( delta_time % 2000000 < 1000000 ) {
				beep_command = DSHOT_CMD_BEEP4;
			}

			if ( beep_command != 0 ) {
				make_packet( 0, beep_command, false );
				make_packet( 1, beep_command, false );
				make_packet( 2, beep_command, false );
				make_packet( 3, beep_command, false );
				dshot_dma_start();
			}
		}
	} else {
		motor_beep_time = 0;
	}
}


#if defined(USE_DSHOT_DMA_DRIVER)

void DMA1_Channel4_5_IRQHandler(void)
{
	DMA_Cmd(DMA1_Channel5, DISABLE);
	DMA_Cmd(DMA1_Channel2, DISABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);

	TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, DISABLE);
	DMA_ClearITPendingBit(DMA1_IT_TC4);
	TIM_Cmd( TIM1, DISABLE );


	switch( dshot_dma_phase ) {
		case 2:
			dshot_dma_phase =1;
			dshot_dma_portB();
			return;
		case 1:
			dshot_dma_phase =0;
			#if defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
				extern int rgb_dma_phase;
				extern void rgb_dma_trigger();

				if( rgb_dma_phase == 2 ) {
					rgb_dma_phase = 1;
					rgb_dma_trigger();
				}
			#endif
			return;
		default :
			dshot_dma_phase =0;
			break;
	}


#if defined(RGB_LED_DMA) && (RGB_LED_NUMBER>0)
	extern int rgb_dma_phase;
	rgb_dma_phase = 0;
#endif

}
#endif

#endif
