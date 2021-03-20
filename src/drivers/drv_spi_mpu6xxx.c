#include "drv_spi_mpu6xxx.h"

#include <stdio.h>

#include "defines.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#ifdef F4

#if defined(ICM20602_SPI_PORT) && defined(ICM20602_NSS)
#define MPU6XXX_SPI_PORT ICM20602_SPI_PORT
#define MPU6XXX_NSS ICM20602_NSS
#ifdef ICM20602_INT
#define MPU6XXX_INT ICM20602_INT
#endif
#endif

#if defined(ICM20601_SPI_PORT) && defined(ICM20601_NSS)
#define MPU6XXX_SPI_PORT ICM20601_SPI_PORT
#define MPU6XXX_NSS ICM20601_NSS
#ifdef ICM20601_INT
#define MPU6XXX_INT ICM20601_INT
#endif
#endif

#if defined(ICM20608_SPI_PORT) && defined(ICM20608_NSS)
#define MPU6XXX_SPI_PORT ICM20608_SPI_PORT
#define MPU6XXX_NSS ICM20608_NSS
#ifdef ICM20608_INT
#define MPU6XXX_INT ICM20608_INT
#endif
#endif

#define PORT spi_port_defs[MPU6XXX_SPI_PORT]

//  Initialize SPI Connection to Gyro
void spi_gyro_init(void) {

  //*********************GPIO**************************************

  spi_init_pins(MPU6XXX_SPI_PORT, MPU6XXX_NSS);

// Interrupt GPIO
#ifdef MPU6XXX_INT
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Mode = GPIO_Mode_IN;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_pin_init(&gpio_init, MPU6XXX_INT);
#endif

  //*********************SPI/DMA**********************************
  spi_enable_rcc(MPU6XXX_SPI_PORT);

  // SPI Config
  SPI_I2S_DeInit(PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(PORT.channel);

  spi_dma_init(MPU6XXX_SPI_PORT);
}

//deinit/reinit spi for unique slave configuration
void spi_MPU6XXX_reinit_slow(void) {
  spi_dma_wait_for_ready(MPU6XXX_SPI_PORT);
  SPI_Cmd(PORT.channel, DISABLE);

  // SPI Config
  SPI_I2S_DeInit(PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
#if defined(ICM20601_SPI_PORT) || defined(ICM20608_SPI_PORT) //5.25mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
#else
#if defined(ICM20602_SPI_PORT) //10mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
#else                          //(MPUXXXX)			 //20mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
#endif
#endif
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);
}

void spi_MPU6XXX_reinit_fast(void) {
  spi_dma_wait_for_ready(MPU6XXX_SPI_PORT);
  SPI_Cmd(PORT.channel, DISABLE);

  // SPI Config
  SPI_I2S_DeInit(PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
#if defined(ICM20601_SPI_PORT) || defined(ICM20608_SPI_PORT) //5.25mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
#else
#if defined(ICM20602_SPI_PORT) //10mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
#else                          //(MPUXXXX)			 //20mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
#endif
#endif
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);
}

//*******************************************************************************SPI / DMA FUNCTIONS********************************************************************************

// blocking dma read of a single register
uint8_t MPU6XXX_dma_spi_read(uint8_t reg) {
  spi_MPU6XXX_reinit_slow();

  uint8_t buffer[2] = {reg | 0x80, 0x00};

  spi_csn_enable(MPU6XXX_NSS);
  spi_dma_transfer_bytes(MPU6XXX_SPI_PORT, buffer, 2);
  spi_csn_disable(MPU6XXX_NSS);

  return buffer[1];
}

// blocking dma write of a single register
void MPU6XXX_dma_spi_write(uint8_t reg, uint8_t data) { //MPU6XXX_dma_spi_write
  spi_MPU6XXX_reinit_slow();

  uint8_t buffer[2] = {reg, data};

  spi_csn_enable(MPU6XXX_NSS);
  spi_dma_transfer_bytes(MPU6XXX_SPI_PORT, buffer, 2);
  spi_csn_disable(MPU6XXX_NSS);
}

void MPU6XXX_dma_read_data(uint8_t reg, int *data, int size) {
  spi_MPU6XXX_reinit_fast();

  uint8_t buffer[15] = {reg | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  spi_csn_enable(MPU6XXX_NSS);
  spi_dma_transfer_bytes(MPU6XXX_SPI_PORT, buffer, size + 1);
  spi_csn_disable(MPU6XXX_NSS);

  for (int i = 1; i < size + 1; i++) {
    data[i - 1] = buffer[i];
  }
}

#endif
