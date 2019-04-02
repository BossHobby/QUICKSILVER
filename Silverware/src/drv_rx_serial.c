//#include "project.h"
#include "uart.h"
//#include <stdio.h>
#include "drv_serial.h"
#include "config.h"
#include "drv_time.h"
#include "defines.h"
#include "drv_rx_serial.h"



//SET SERIAL BAUDRATE BASED ON RECEIVER PROTOCOL

#if defined (RX_DSMX_2048) || defined (RX_DSM2_1024)
//#include "rx_dsm.h"
#define SERIAL_BAUDRATE 115200
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

//FUNCTION TO INITIALIZE USART FOR A SERIAL RX CALLED FROM RECEIVER PROTOCOL

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF)
void usart_rx_init(void)
{
    // make sure there is some time to program the board if SDA pins are reinitialized as GPIO
    if ( gettime() < 2000000 ) return;    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;   
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Pin = SERIAL_RX_PIN;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure); 
    GPIO_PinAFConfig(SERIAL_RX_PORT, SERIAL_RX_SOURCE , SERIAL_RX_CHANNEL);
	
  //  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
		APBPeriphClockCmd();
	
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;    //sbus is even parity
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx ;//USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SERIAL_RX_USART, &USART_InitStructure);
// swap rx/tx pins - available on F0 targets
#ifdef F0_USART_PINSWAP
    USART_SWAPPinCmd( SERIAL_RX_USART, ENABLE);
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


