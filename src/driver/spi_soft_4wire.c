#include "driver/spi_soft.h"
#include "project.h"

#ifdef USE_SOFT_SPI_4WIRE

void spi_init() {
  // spi port inits

  LL_GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = SPI_MOSI_PIN;
  LL_GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = SPI_CLK_PIN;
  LL_GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = SPI_SS_PIN;
  LL_GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

  // miso should be input by default

  spi_csoff();
}

#define MOSIHIGH LL_GPIO_SetOutputPin(SPI_MOSI_PORT, SPI_MOSI_PIN);
#define MOSILOW LL_GPIO_ResetOutputPin(SPI_MOSI_PORT, SPI_MOSI_PIN);
#define SCKHIGH LL_GPIO_SetOutputPin(SPI_CLK_PORT, SPI_CLK_PIN);
#define SCKLOW LL_GPIO_ResetOutputPin(SPI_CLK_PORT, SPI_CLK_PIN);

#define READMISO LL_GPIO_IsInputPinSet(SPI_MISO_PORT, SPI_MISO_PIN)

void spi_cson() {
  LL_GPIO_ResetOutputPin(SPI_SS_PORT, SPI_SS_PIN);
}

void spi_csoff() {
  LL_GPIO_SetOutputPin(SPI_SS_PORT, SPI_SS_PIN);
}

void spi_sendbyte(int data) {
  for (int i = 7; i >= 0; i--) {
    if ((data >> i) & 1) {
      MOSIHIGH;
    } else {
      MOSILOW;
    }

    SCKHIGH;

    SCKLOW;
  }
}

int spi_sendrecvbyte2(int data) {
  int recv = 0;
  for (int i = 7; i >= 0; i--) {
    if ((data) & (1 << 7)) {
      MOSIHIGH;
    } else {
      MOSILOW;
    }

    SCKHIGH;

    data = data << 1;
    if (READMISO)
      recv = recv | (1 << 7);
    recv = recv << 1;
    SCKLOW;
  }
  recv = recv >> 8;
  return recv;
}

int spi_sendrecvbyte(int data) {
  int recv = 0;

  for (int i = 7; i >= 0; i--) {
    recv = recv << 1;
    if ((data) & (1 << 7)) {
      MOSIHIGH;
    } else {
      MOSILOW;
    }

    data = data << 1;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    SCKHIGH;

    if (READMISO)
      recv = recv | 1;

    SCKLOW;
  }

  return recv;
}

int spi_sendzerorecvbyte() {
  int recv = 0;
  MOSILOW;

  for (int i = 7; i >= 0; i--) {
    recv = recv << 1;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    SCKHIGH;

    if (READMISO)
      recv = recv | 1;

    SCKLOW;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
  }
  return recv;
}

#endif
