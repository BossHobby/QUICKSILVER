#include "drv_dma_spi.h"

#include <stdio.h>

#include "defines.h"
#include "drv_time.h"
#include "project.h"

#ifdef F405

#if defined(ICM20601_SPI1) || defined(ICM20602_SPI1)
#define MPU6XXX_SPI1
#endif

#if defined(ICM20601_SPI2) || defined(ICM20602_SPI2)
#define MPU6XXX_SPI2
#endif

//SPI PINS
#ifdef MPU6XXX_SPI1
#define RCC_AHB1Periph_DMA RCC_AHB1Periph_DMA2
#define RCC_APBPeriph_SPI RCC_APB2Periph_SPI1
#define MPU6XXX_SPI_INSTANCE SPI1
#define MPU6XXX_SPI_PORT GPIOA
#define MPU6XXX_SCLK_PINSOURCE GPIO_PinSource5
#define MPU6XXX_SCLK_PIN GPIO_Pin_5
#define MPU6XXX_MISO_PINSOURCE GPIO_PinSource6
#define MPU6XXX_MISO_PIN GPIO_Pin_6
#define MPU6XXX_MOSI_PINSOURCE GPIO_PinSource7
#define MPU6XXX_MOSI_PIN GPIO_Pin_7
#define MPU6XXX_SPI_AF GPIO_AF_SPI1
#define DMA_RX_STREAM DMA2_Stream2
#define DMA_TX_STREAM DMA2_Stream3
#define DMA_RX_CHANNEL DMA_Channel_3
#define DMA_TX_CHANNEL DMA_Channel_3
#define DMA_RX_TCI_FLAG DMA_FLAG_TCIF2
#define DMA_TX_TCI_FLAG DMA_FLAG_TCIF3
#define DMA_STREAM_IRQ DMA2_Stream2_IRQn
#define DMA_RX_IT_FLAG DMA_IT_TCIF2
#endif

#ifdef MPU6XXX_SPI2
#define RCC_AHB1Periph_DMA RCC_AHB1Periph_DMA1
#define RCC_APBPeriph_SPI RCC_APB1Periph_SPI2
#define MPU6XXX_SPI_INSTANCE SPI2
#define MPU6XXX_SPI_PORT GPIOB
#define MPU6XXX_SCLK_PINSOURCE GPIO_PinSource13
#define MPU6XXX_SCLK_PIN GPIO_Pin_13
#define MPU6XXX_MISO_PINSOURCE GPIO_PinSource14
#define MPU6XXX_MISO_PIN GPIO_Pin_14
#define MPU6XXX_MOSI_PINSOURCE GPIO_PinSource15
#define MPU6XXX_MOSI_PIN GPIO_Pin_15
#define MPU6XXX_SPI_AF GPIO_AF_SPI2
#define DMA_RX_STREAM DMA1_Stream3
#define DMA_TX_STREAM DMA1_Stream4
#define DMA_RX_CHANNEL DMA_Channel_0
#define DMA_TX_CHANNEL DMA_Channel_0
#define DMA_RX_TCI_FLAG DMA_FLAG_TCIF3
#define DMA_TX_TCI_FLAG DMA_FLAG_TCIF4
#define DMA_STREAM_IRQ DMA1_Stream3_IRQn
#define DMA_RX_IT_FLAG DMA_IT_TCIF3
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

#if defined(MPU6XXX_NSS_PC2) || defined(ICM20601_NSS_PC2) || defined(ICM20602_NSS_PC2)
#define MPU6XXX_NSS_PINSOURCE GPIO_PinSource2
#define MPU6XXX_NSS_PIN GPIO_Pin_2
#define MPU6XXX_NSS_PORT GPIOC
#endif

#if defined(MPU6XXX_NSS_PB12) || defined(ICM20601_NSS_PB12) || defined(ICM20602_NSS_PB12)
#define MPU6XXX_NSS_PINSOURCE GPIO_PinSource12
#define MPU6XXX_NSS_PIN GPIO_Pin_12
#define MPU6XXX_NSS_PORT GPIOB
#endif

//INTERRUPT PINS
#if defined(MPU6XXX_INT_PC4) || defined(ICM20601_INT_PC4) || defined(ICM20602_INT_PC4)
#define MPU6XXX_INT_PIN GPIO_Pin_4
#define MPU6XXX_INT_PORT GPIOC
#endif

#if defined(MPU6XXX_INT_PC3) || defined(ICM20601_INT_PC3) || defined(ICM20602_INT_PC3)
#define MPU6XXX_INT_PIN GPIO_Pin_3
#define MPU6XXX_INT_PORT GPIOC
#endif

#if defined(MPU6XXX_INT_PA8) || defined(ICM20601_INT_PA8) || defined(ICM20602_INT_PA8)
#define MPU6XXX_INT_PIN GPIO_Pin_8
#define MPU6XXX_INT_PORT GPIOA
#endif


//  Initialize SPI Connection to Gyro
void spi_gyro_init(void) {

  //*********************GPIO**************************************

  // GPIO & Alternate Function Setting
  GPIO_InitTypeDef GPIO_InitStructure;
  // Clock, Miso, Mosi GPIO
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_SCLK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MPU6XXX_SPI_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MPU6XXX_MOSI_PIN | MPU6XXX_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MPU6XXX_SPI_PORT, &GPIO_InitStructure);
/*
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_MISO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MPU6XXX_SPI_PORT, &GPIO_InitStructure);
*/
  // Chip Select GPIO
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MPU6XXX_NSS_PORT, &GPIO_InitStructure);

  // Interrupt GPIO
  #ifdef MPU6XXX_INT_PIN
  GPIO_InitStructure.GPIO_Pin = MPU6XXX_INT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MPU6XXX_INT_PORT, &GPIO_InitStructure);
  #endif

  // Chip Select Set High
  GPIO_SetBits(MPU6XXX_NSS_PORT, MPU6XXX_NSS_PIN);

  // Connect SPI pins to AF_SPI1
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_SCLK_PINSOURCE, MPU6XXX_SPI_AF); //SCLK
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_MISO_PINSOURCE, MPU6XXX_SPI_AF); //MISO
  GPIO_PinAFConfig(MPU6XXX_SPI_PORT, MPU6XXX_MOSI_PINSOURCE, MPU6XXX_SPI_AF); //MOSI

  //*********************SPI/DMA**********************************
  #if defined(MPU6XXX_SPI1) || defined(MPU6XXX_SPI4)
  //SPI1 to APB2 bus clock
  RCC_APB2PeriphClockCmd(RCC_APBPeriph_SPI, ENABLE);
  #endif
  #if defined(MPU6XXX_SPI2) || defined(MPU6XXX_SPI3)
  //SPI2 to APB1 bus clock
  RCC_APB1PeriphClockCmd(RCC_APBPeriph_SPI, ENABLE);
  #endif
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
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);

  // Enable DMA clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA, ENABLE);

  // Enable DMA Interrupt on receive line
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = DMA_STREAM_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStruct);
}

// dma stream inits
void dma_receive_MPU6XXX_init(uint8_t *base_address_in, uint8_t buffer_size) {
  DMA_InitTypeDef DMA_InitStructure;

  //RX Stream
  DMA_DeInit(DMA_RX_STREAM);
  DMA_InitStructure.DMA_Channel = DMA_RX_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(MPU6XXX_SPI_INSTANCE->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_in;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  ;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA_RX_STREAM, &DMA_InitStructure);
}

void dma_transmit_MPU6XXX_init(uint8_t *base_address_out, uint8_t buffer_size) {
  DMA_InitTypeDef DMA_InitStructure;

  //TX Stream
  DMA_DeInit(DMA_TX_STREAM);
  DMA_InitStructure.DMA_Channel = DMA_TX_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(MPU6XXX_SPI_INSTANCE->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_out;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA_TX_STREAM, &DMA_InitStructure);
}

//deinit/reinit spi for unique slave configuration
void spi_MPU6XXX_reinit_slow(void) {
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
	  #else	//(MPUXXXX)			 //20mhz SPI
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	  #endif
	  #endif
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_Init(MPU6XXX_SPI_INSTANCE, &SPI_InitStructure);
}

void spi_MPU6XXX_reinit_fast(void) {
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
	  #else	//(MPUXXXX)			 //20mhz SPI
	  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	  #endif
	  #endif
	  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	  SPI_InitStructure.SPI_CRCPolynomial = 7;
	  SPI_Init(MPU6XXX_SPI_INSTANCE, &SPI_InitStructure);
}


//*******************************************************************************SPI / DMA FUNCTIONS********************************************************************************

extern int liberror; //tracks any failed spi reads or writes to trigger failloop		//this really isn't used with dma anymore

// Chip Select functions
void spi_enable(void) {
  GPIO_ResetBits(MPU6XXX_NSS_PORT, MPU6XXX_NSS_PIN);
}
void spi_disable(void) {
  GPIO_SetBits(MPU6XXX_NSS_PORT, MPU6XXX_NSS_PIN);
}

//blocking dma transmit bytes
void MPU6XXX_dma_transfer_bytes(uint8_t *buffer, uint8_t length) {
  dma_receive_MPU6XXX_init(buffer, length);
  dma_transmit_MPU6XXX_init(buffer, length);
  spi_enable();
  DMA_Cmd(DMA_RX_STREAM, ENABLE); // Enable the DMA SPI RX Stream
  DMA_Cmd(DMA_TX_STREAM, ENABLE); // Enable the DMA SPI TX Stream
  // Enable the SPI Rx/Tx DMA request
  SPI_I2S_DMACmd(MPU6XXX_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(MPU6XXX_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, ENABLE);
  /* Waiting the end of Data transfer */
  while (DMA_GetFlagStatus(DMA_RX_STREAM, DMA_RX_TCI_FLAG) == RESET) {
  };
  while (DMA_GetFlagStatus(DMA_TX_STREAM, DMA_TX_TCI_FLAG) == RESET) {
  };
  DMA_ClearFlag(DMA_RX_STREAM, DMA_RX_TCI_FLAG);
  DMA_ClearFlag(DMA_TX_STREAM, DMA_TX_TCI_FLAG);
  DMA_Cmd(DMA_TX_STREAM, DISABLE);
  DMA_Cmd(DMA_RX_STREAM, DISABLE);
  SPI_I2S_DMACmd(MPU6XXX_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(MPU6XXX_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, DISABLE);
  spi_disable();
}

// blocking dma read of a single register
uint8_t MPU6XXX_dma_spi_read(uint8_t reg) {
  uint8_t buffer[2] = {reg | 0x80, 0x00};
  spi_MPU6XXX_reinit_slow();
  MPU6XXX_dma_transfer_bytes(buffer, 2);
  return buffer[1];
}

// blocking dma write of a single register
void MPU6XXX_dma_spi_write(uint8_t reg, uint8_t data) { //MPU6XXX_dma_spi_write
  uint8_t buffer[2] = {reg, data};
  spi_MPU6XXX_reinit_slow();
  MPU6XXX_dma_transfer_bytes(buffer, 2);
}

void MPU6XXX_dma_read_data(uint8_t reg, int *data, int size) {
  uint8_t buffer[15] = {reg | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  spi_MPU6XXX_reinit_fast();
  MPU6XXX_dma_transfer_bytes(buffer, size + 1);
  for (int i = 1; i < size + 1; i++) {
    data[i-1] = buffer[i];
  }
}


//****************************************************************************LEGACY SPI FUNCTIONS********************************************************************************

/*
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
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  #endif
  #endif
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(MPU6XXX_SPI_INSTANCE, &SPI_InitStructure);
  SPI_Cmd(MPU6XXX_SPI_INSTANCE, ENABLE);
  // Dummy read to clear receive buffer - just in case
  while (SPI_I2S_GetFlagStatus(MPU6XXX_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET);
  SPI_I2S_ReceiveData(MPU6XXX_SPI_INSTANCE);
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
*/
//**************************************************************************************************************************************************************************

//TODO: Determine any advantage to interrupt based non blocking gyro read.  Current blocking read takes ~8.5us
// Transfer Complete Callback Function
// IRQ handler

#endif
