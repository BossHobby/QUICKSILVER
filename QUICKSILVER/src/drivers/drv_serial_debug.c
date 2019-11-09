// serial for stm32 not used yet
#include "drv_serial.h"
#include "defines.h"
#include "drv_uart.h"
#include "project.h"
#include <stdio.h>

// enable serial driver ( pin SWCLK after calibration)
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)

// this has to be in config.h
//#define SERIAL_ENABLE

#define SERIAL_BUFFER_SIZE 64

#define SERIAL_BAUDRATE 115200

#ifdef SERIAL_ENABLE

uint8_t buffer[SERIAL_BUFFER_SIZE];
char buffer_start = 0;
char buffer_end = 0;

void USART1_IRQHandler(void) {
  if (buffer_end != buffer_start) {
    USART_SendData(USART1, buffer[buffer_start]);
    buffer_start++;
    buffer_start = buffer_start % (SERIAL_BUFFER_SIZE);
  } else {
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  }
}

void serial_init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_1);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx; //USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure);

  //	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_Cmd(USART1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

int fputc(int ch, FILE *f) {
  buffer[buffer_end] = (char)ch;
  buffer_end++;
  buffer_end = buffer_end % (SERIAL_BUFFER_SIZE);

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  return ch;
}

void buffer_add(int val) {
  buffer[buffer_end] = (char)val;
  buffer_end++;
  buffer_end = buffer_end % (SERIAL_BUFFER_SIZE);

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  return;
}

#else
// serial disabled - dummy functions
void serial_init(void) {
}

void buffer_add(int val) {
}

#endif
