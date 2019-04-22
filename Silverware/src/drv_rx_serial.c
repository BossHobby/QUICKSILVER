//#include "project.h"
#include "uart.h"
//#include <stdio.h>
#include "drv_serial.h"
#include "config.h"
#include "drv_time.h"
#include "defines.h"
#include "drv_rx_serial.h"


//SET SERIAL BAUDRATE BASED ON RECEIVER PROTOCOL

#if defined (RX_DSMX_2048) || defined (RX_DSM2_1024) || defined(RX_IBUS)
#define SERIAL_BAUDRATE 115200
#endif
#if defined (RX_SBUS)
#define SERIAL_BAUDRATE 100000
#endif

//FUNCTION TO SET APB CLOCK TO USART BASED ON USER SELECTED UART, TARGET MCU, AND TARGET DEFINED USART ALTERNATE FUNCTION PINS CALLED BELOW FROM INSIDE usart_rx_init()

void APBPeriphClockCmd(void)
{
#if defined (UART_1) && defined (F0)
#if defined (USART1_PA3PA2) || defined (USART1_SDA)
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#endif

#if defined (UART_1) && defined (F405)
#ifdef USART1_PA10PA9
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#endif
#endif

#if defined (UART_3) && defined (F405)
#ifdef USART3_PB11PB10
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif
#endif
}


//FUNCTION TO COMMAND EXTERNAL USART INVERTER HIGH OR LOW         todo: sort out target mapping tag in drv_rx_serial.h for a quick define from the taarget

#define USART_INVERTER_PIN GPIO_Pin_0   //hard defined here for testing right now on cc3d revo
#define USART_INVERTER_PORT GPIOC

void usart_invert(void)
{
	#ifdef F405
        GPIO_InitTypeDef    GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = USART_INVERTER_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(USART_INVERTER_PORT, &GPIO_InitStructure); 
        #ifdef INVERT_UART
        // Inverter control line, set high
        GPIO_SetBits(USART_INVERTER_PORT, USART_INVERTER_PIN);
				#else          
        // Inverter control line, set low
        GPIO_ResetBits(USART_INVERTER_PORT, USART_INVERTER_PIN);
        #endif  
	#else
	// do nothing here, usart swap command in usart init
	#endif
}


//FUNCTION TO INITIALIZE USART FOR A SERIAL RX CALLED FROM RECEIVER PROTOCOL

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS)
void usart_rx_init(void)
{
    // make sure there is some time to program the board if SDA pins are reinitialized as GPIO
    if ( gettime() < 2000000 ) return;    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
#if defined (RX_DSMX_2048) || defined (RX_DSM2_1024) || defined (RX_CRSF) || defined (RX_IBUS)
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
#endif
#if defined (RX_SBUS)
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Pin = SERIAL_RX_PIN;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure); 
    GPIO_PinAFConfig(SERIAL_RX_PORT, SERIAL_RX_SOURCE , SERIAL_RX_CHANNEL);
		APBPeriphClockCmd();	
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
#if defined (RX_DSMX_2048) || defined (RX_DSM2_1024) || defined (RX_CRSF) || defined (RX_IBUS)	
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;    //sbus is even parity
#endif
#if defined (RX_SBUS)
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;   //todo: try setting even
#endif
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx ;//USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SERIAL_RX_USART, &USART_InitStructure);
#ifdef F0
#ifdef INVERT_UART
		USART_InvPinCmd(SERIAL_RX_USART, USART_InvPin_Rx|USART_InvPin_Tx , ENABLE );
#endif
// swap rx/tx pins - available on F0 targets
#ifdef F0_USART_PINSWAP
    USART_SWAPPinCmd( SERIAL_RX_USART, ENABLE);
#endif
#endif
    USART_ITConfig(SERIAL_RX_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(SERIAL_RX_USART, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_USART_IRQ;
#ifdef F405
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
#else
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
#endif
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
#endif


//USART ISR to radio protocol mapping
#ifdef UART_1
void USART1_IRQHandler(void)
{
	#ifdef RX_IBUS 
	Ibus_USART_ISR();
	#endif
	#if defined RX_DSMX_2048 || defined RX_DSM2_1024
	DSM_USART_ISR();
	#endif
	#ifdef RX_SBUS 
	SBUS_USART_ISR();
	#endif
	}
#endif
#ifdef UART_3
void USART3_IRQHandler(void)
{
	#ifdef RX_IBUS 
	Ibus_USART_ISR();
	#endif
	#if defined RX_DSMX_2048 || defined RX_DSM2_1024
	DSM_USART_ISR();
	#endif
	#ifdef RX_SBUS 
	SBUS_USART_ISR();
	#endif
}
	#endif
