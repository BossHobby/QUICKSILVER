#include "drv_spi.h"

#include "project.h"

#ifdef SOFTSPI_4WIRE

void spi_init(void) {
  // spi port inits

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SPI_CLK_PIN;
  GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SPI_SS_PIN;
  GPIO_Init(SPI_SS_PORT, &GPIO_InitStructure);

  //miso should be input by default

  spi_csoff();
}

#ifdef F4
#define gpioset(port, pin) port->BSRRL = pin
#define gpioreset(port, pin) port->BSRRH = pin
#endif

#define MOSIHIGH gpioset(SPI_MOSI_PORT, SPI_MOSI_PIN)
#define MOSILOW gpioreset(SPI_MOSI_PORT, SPI_MOSI_PIN);
#define SCKHIGH gpioset(SPI_CLK_PORT, SPI_CLK_PIN);
#define SCKLOW gpioreset(SPI_CLK_PORT, SPI_CLK_PIN);

//#define READMISO (GPIO_ReadInputDataBit(SPI_MISO_PORT, SPI_MISO_PIN) )
#define READMISO (SPI_MISO_PORT->IDR & SPI_MISO_PIN)

//#pragma push

//#pragma Otime
//#pragma O2

void spi_cson() {
#ifdef F4
  SPI_SS_PORT->BSRRH = SPI_SS_PIN;
#endif
}

void spi_csoff() {
#ifdef F4
  SPI_SS_PORT->BSRRL = SPI_SS_PIN;
#endif
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
#ifdef F4
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
#ifdef F4
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
#ifdef F4
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

//#pragma pop

#endif
