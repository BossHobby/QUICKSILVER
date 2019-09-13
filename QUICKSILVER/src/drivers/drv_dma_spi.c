#include "drv_dma_spi.h"

#include <stdio.h>

#include "defines.h"
#include "drv_time.h"
#include "project.h"

#ifdef F405

#if defined(ICM20601_SPI1) || defined(ICM20602_SPI1)
#define MPU6XXX_SPI1
#endif

//SPI PINS
#ifdef MPU6XXX_SPI1
#define MPU6XXX_SPI_INSTANCE SPI1
#define MPU6XXX_SPI_PORT GPIOA
#define MPU6XXX_SCLK_PINSOURCE GPIO_PinSource5
#define MPU6XXX_SCLK_PIN GPIO_Pin_5
#define MPU6XXX_MISO_PINSOURCE GPIO_PinSource6
#define MPU6XXX_MISO_PIN GPIO_Pin_6
#define MPU6XXX_MOSI_PINSOURCE GPIO_PinSource7
#define MPU6XXX_MOSI_PIN GPIO_Pin_7
#define MPU6XXX_SPI_AF GPIO_AF_SPI1
#endif

//CHIP SELECT PINS
#if defined(MPU6XXX_NSS_PA8) || defined(ICM20601_NSS_PA8) || defined(ICM20602_NSS_PA8)
#define MPU6XXX_NSS_PINSOURCE GPIO_PinSource8
#define MPU6XXX_NSS_PIN GPIO_Pin_8
#define MPU6XXX_NSS_PORT GPIOA
#endif

#if defined(MPU6XXX_NSS_PA4) || defined(ICM20601_NSS_PA4) || defined(ICM20602_NSS_PA4)
#define MPU6XXX_NSS_PINSOURCE GPIO_PinSource4
#define MPU6XXX_NSS_PIN GPIO_Pin_4
#define MPU6XXX_NSS_PORT GPIOA
#endif

//INTERRUPT PINS
#if defined(MPU6XXX_INT_PC4) || defined(ICM20601_INT_PC4) || defined(ICM20602_INT_PC4)
#define MPU6XXX_INT_PIN GPIO_Pin_4
#define MPU6XXX_INT_PORT GPIOC
#endif

//  Initialize SPI Connection to Gyro
void spi_gyro_init(void) {
  // RCC Clock Setting
  //RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);	// this doesnt need to be turned on till DMA is coded
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //this clock is already on from gpio driver

  //*********************GPIO**************************************

  // GPIO & Alternate Function Setting
  GPIO_InitTypeDef GPIO_InitStructure;
  // Clock, Miso, Mosi GPIO
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_SCLK_PIN | MPU6XXX_MISO_PIN | MPU6XXX_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(MPU6XXX_SPI_PORT, &GPIO_InitStructure);

  // Chip Select GPIO
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MPU6XXX_NSS_PORT, &GPIO_InitStructure);

  // Interrupt GPIO
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_INT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MPU6XXX_INT_PORT, &GPIO_InitStructure);

  // Chip Select Set High
  GPIO_SetBits(MPU6XXX_SPI_PORT, MPU6XXX_NSS_PIN);

  // TODO: GPIO Listen to interrupt pin, set up interrupt handler, ect

  // Connect SPI pins to AF_SPI1
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_SCLK_PINSOURCE, MPU6XXX_SPI_AF); //SCLK
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_MISO_PINSOURCE, MPU6XXX_SPI_AF); //MISO
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_MOSI_PINSOURCE, MPU6XXX_SPI_AF); //MOSI

  //*********************SPI***************************************

  //SPI1 to APB2 bus clock																					//TODO  Make this populate with defines for switching SPI instances
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  // SPI Config
  SPI_I2S_DeInit(MPU6XXX_SPI_INSTANCE);
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
  SPI_Init(MPU6XXX_SPI_INSTANCE, &SPI_InitStructure);
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
}

//*********************FUNCTIONS************************************
extern int liberror; //tracks any failed spi reads or writes to trigger failloop

// Reset spi prescaler to 5.25mhz, 10mhz, or 20mhz
void spi_reset_prescaler(void) {
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, DISABLE);
  // SPI Config
  SPI_I2S_DeInit(MPU6XXX_SPI_INSTANCE);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
#if defined(ICM20601_SPI1) //5.25mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
#else
#if defined(ICM20602_SPI1) //10mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
#else                      //(MPUXXXX)				//20mhz SPI
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
#endif
#endif
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(MPU6XXX_SPI_INSTANCE, &SPI_InitStructure);
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, ENABLE);
  // Dummy read to clear receive buffer - just in case
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
}

void spi_reset_prescaler2(void) {
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, DISABLE);
  const uint16_t clearBRP = 0xFFC7;
  uint16_t temp = MPU6XXX_SPI_INSTANCE->CR1;
  temp &= clearBRP;
  temp |= SPI_BaudRatePrescaler_2;
  MPU6XXX_SPI_INSTANCE->CR1 = temp;
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, ENABLE);
}

// Chip Select functions
void spi_enable(void) {
  GPIO_ResetBits(MPU6XXX_SPI_PORT, MPU6XXX_NSS_PIN);
}
void spi_disable(void) {
  GPIO_SetBits(MPU6XXX_SPI_PORT, MPU6XXX_NSS_PIN);
}

// Blocking Transmit/Read function
uint8_t spi_transfer_byte(uint8_t data) {
  uint16_t spiTimeout;

  //check if transmit buffer empty flag is set
  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      break;
    }
  }

  // Send out data
  SPI_I2S_SendData(MPU6XXX_SPI_INSTANCE, data);

  //wait to receive something ... timeout if nothing comes in
  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      break;
    }
  }

  // Return back received data in SPIx->DR
  return SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
}

// Function to write gyro registers
uint8_t MPU6XXX_write(uint8_t reg, uint8_t data) {
  uint8_t stuff;
  spi_enable();
  stuff = spi_transfer_byte(reg | 0x00);
  stuff = spi_transfer_byte(data);
  spi_disable();
  return stuff;
}

// Function to read gyro registers
uint8_t MPU6XXX_read(uint8_t reg) {
  uint8_t stuff;
  spi_enable();
  spi_transfer_byte(reg | 0x80);
  stuff = spi_transfer_byte(0x00);
  spi_disable();
  return stuff;
}

//Function to read gyro motion data registers
void MPU6XXX_read_data(uint8_t reg, int *data, int size) {
  spi_enable();
  spi_transfer_byte(reg | 0x80);
  for (int i = 0; i < size; i++) {
    data[i] = spi_transfer_byte(0x00);
  }
  spi_disable();
}

////////////////////////////////WORKING UP TO HERE SO FAR//////////////////////////////////////////////////////////////

// Initialize DMA transmit and receive streams to SPI and bring SPI up to full speed for bulk transfer
void dma_spi_init(void) {
  // Speed up SPI

  //TODO

  // Configure the DMA
  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = DMA_Channel_3;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
}

// Transfer Complete Callback Function

// IRQ handler

#endif
