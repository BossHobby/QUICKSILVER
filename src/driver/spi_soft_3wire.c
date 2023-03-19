#include "core/project.h"
#include "driver/spi_soft.h"
#include "driver/time.h"

#ifdef USE_SOFT_SPI_3WIRE

LL_GPIO_InitTypeDef mosi_init_struct;
int mosi_out = 0;

void spi_init() {
  LL_GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = SPI_CLK_PIN;
  LL_GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = SPI_SS_PIN;
  LL_GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

  mosi_init_struct.Pin = SPI_MOSI_PIN;
  mosi_init_struct.Mode = LL_GPIO_MODE_INPUT;
  mosi_init_struct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  mosi_init_struct.Pull = LL_GPIO_PULL_UP;
  mosi_init_struct.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  LL_GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);

  // mosi_out = 0; // already

  spi_csoff();
}

void mosi_input() {
  if (mosi_out) {
    mosi_out = 0;
    mosi_init_struct.Mode = LL_GPIO_MODE_INPUT;
    LL_GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
  }
}

void mosi_output() {
  if (!mosi_out) {
    mosi_out = 1;
    mosi_init_struct.Mode = LL_GPIO_MODE_OUTPUT;
    LL_GPIO_Init(SPI_MOSI_PORT, &mosi_init_struct);
  }
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
  mosi_output();
  for (int i = 7; i >= 0; i--) {
    if ((data >> i) & 1) {
      MOSIHIGH;
    } else {
      MOSILOW;
    }
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    SCKHIGH;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    SCKLOW;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
  }
}

int spi_recvbyte() {
  int recv = 0;

  for (int i = 7; i >= 0; i--) {
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    SCKHIGH;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
    __asm("NOP");
#endif
    recv = recv << 1;

    recv = recv | ((SPI_MOSI_PORT->IDR & (int)SPI_MOSI_PIN) ? 1 : 0);

    SCKLOW;
#ifdef STM32F4
    __asm("NOP");
    __asm("NOP");
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

int spi_recvbyte_unrolled() {
  uint8_t recv = 0;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  SCKHIGH;

  recv = recv << 1;

  recv = recv | ((SPI_MOSI_PORT->IDR & SPI_MOSI_PIN) ? 1 : 0);

  SCKLOW;

  return recv;
}

#endif
