#include "drv_spi_mpu6xxx.h"

#include <stdio.h>
#include <stm32f4xx_ll_spi.h>

#include "defines.h"
#include "drv_gpio.h"
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
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_INPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_pin_init(&gpio_init, MPU6XXX_INT);
#endif

  //*********************SPI/DMA**********************************
  spi_enable_rcc(MPU6XXX_SPI_PORT);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);

  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET)
    ;
  LL_SPI_ReceiveData8(PORT.channel);

  spi_dma_init(MPU6XXX_SPI_PORT);
}

//deinit/reinit spi for unique slave configuration
void spi_MPU6XXX_reinit_slow(void) {
  spi_dma_wait_for_ready(MPU6XXX_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
#if defined(ICM20601_SPI_PORT) || defined(ICM20608_SPI_PORT) //5.25mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
#else
#if defined(ICM20602_SPI_PORT) //10mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
#else                          //(MPUXXXX)			 //20mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV64;
#endif
#endif
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
}

void spi_MPU6XXX_reinit_fast(void) {
  spi_dma_wait_for_ready(MPU6XXX_SPI_PORT);
  LL_SPI_Disable(PORT.channel);

  // SPI Config
  LL_SPI_DeInit(PORT.channel);
  LL_SPI_InitTypeDef spi_init;
  spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  spi_init.Mode = LL_SPI_MODE_MASTER;
  spi_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  spi_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  spi_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  spi_init.NSS = LL_SPI_NSS_SOFT;
#if defined(ICM20601_SPI_PORT) || defined(ICM20608_SPI_PORT) //5.25mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
#else
#if defined(ICM20602_SPI_PORT) //10mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
#else                          //(MPUXXXX)			 //20mhz SPI
  spi_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
#endif
#endif
  spi_init.BitOrder = LL_SPI_MSB_FIRST;
  spi_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  spi_init.CRCPoly = 7;
  LL_SPI_Init(PORT.channel, &spi_init);

  LL_SPI_Enable(PORT.channel);
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
