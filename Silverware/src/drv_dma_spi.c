#include "project.h"
#include <stdio.h>
#include "config.h"
#include "drv_time.h"






//  Initialize SPI Connection to Gyro
void spi_gyro_init(void)
{
// RCC Clock Setting
//RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA2, ENABLE);
//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); this clock is already on from gpio driver
//*********************GPIO**************************************	
	
// GPIO & Alternate Function Setting
GPIO_InitTypeDef    GPIO_InitStructure;
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
GPIO_Init(MPU6XXX_SPI_PORT, &GPIO_InitStructure);

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
RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1, ENABLE);
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

while(SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET) ;
SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
}


// Chip Select functions
void spi_enable(void)
{
	GPIO_ResetBits(MPU6XXX_SPI_PORT, MPU6XXX_NSS_PIN);	
}
void spi_disable(void)
{
	GPIO_SetBits(MPU6XXX_SPI_PORT, MPU6XXX_NSS_PIN);	
}


// Blocking Transmit/Read function
uint8_t spi_transfer_byte(uint8_t data)  //blocking send using spi to configure gyro
{
  uint8_t byte[2];
	byte[0] = data;
  uint16_t spiTimeout;
	
  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return 0;
  }
	// Send data
  SPI_I2S_SendData(MPU6XXX_SPI_INSTANCE, data);

  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return 0;
  }

  // Pack received data into the same variable
  byte[1] = (uint8_t)SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
  return byte[1];

}


// Function to write gyro registers
void MPU6XXX_write(uint8_t reg, uint8_t data)  //TODO:  deal with fail spi timeout return 0 case
{
  spi_enable();
  spi_transfer_byte(reg);
  spi_transfer_byte(data);
  spi_disable();
}

uint8_t MPU6XXX_get_id(uint8_t reg)
{
	spi_enable();
	uint8_t byte[2];
	byte[0] = reg;
  uint16_t spiTimeout;
	
  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return 0;
  }
	// Send data
  SPI_I2S_SendData(MPU6XXX_SPI_INSTANCE, reg);

  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if ((spiTimeout--) == 0)
      return 0;
  }

  // Pack received data into the same variable
  byte[1] = (uint8_t)SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
  return byte[1];
	spi_disable();
}

// Initialize DMA transmit and receive streams to SPI and bring SPI up to full speed for bulk transfer 
void dma_spi_init(void)
{
// Speed up SPI

		//TODO
	
// Configure the DMA
DMA_InitTypeDef     DMA_InitStructure;
DMA_InitStructure.DMA_Channel = DMA_Channel_3;
DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
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





