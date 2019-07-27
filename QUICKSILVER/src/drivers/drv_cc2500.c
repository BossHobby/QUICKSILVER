#include "drv_cc2500.h"

#include "project.h"

#if defined(F405) && defined(USE_CC2500)

#ifdef CC2500_SPI3
#define CC2500_SPI_INSTANCE SPI3
#define CC2500_SPI_PORT GPIOB

#define CC2500_SCLK_PINSOURCE GPIO_PinSource3
#define CC2500_SCLK_PIN GPIO_Pin_3

#define CC2500_MISO_PINSOURCE GPIO_PinSource4
#define CC2500_MISO_PIN GPIO_Pin_4

#define CC2500_MOSI_PINSOURCE GPIO_PinSource5
#define CC2500_MOSI_PIN GPIO_Pin_5

#define CC2500_SPI_AF GPIO_AF_SPI3
#endif

#ifdef CC2500_NSS_PA15
#define CC2500_NSS_PINSOURCE GPIO_PinSource15
#define CC2500_NSS_PIN GPIO_Pin_15
#define CC2500_NSS_PORT GPIOA
#endif

#ifdef CC2500_GDO0_PC14
#endif

#ifdef CC2500_TX_EN_PA8
#endif

#ifdef CC2500_LNA_EN_PA13
#endif

#ifdef CC2500_ANT_SEL_PA14
#endif

extern int liberror;

void cc2500_csn_enable() {
  GPIO_ResetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);
}

void cc2500_csn_disable() {
  GPIO_SetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);
}

void cc2500_hardware_init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  // SCLK, MISO, MOSI
  GPIO_InitStructure.GPIO_Pin = CC2500_SCLK_PIN | CC2500_MISO_PIN | CC2500_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(CC2500_SPI_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(CC2500_SPI_PORT, CC2500_SCLK_PINSOURCE, CC2500_SPI_AF); //SCLK
  GPIO_PinAFConfig(CC2500_SPI_PORT, CC2500_MISO_PINSOURCE, CC2500_SPI_AF); //MISO
  GPIO_PinAFConfig(CC2500_SPI_PORT, CC2500_MOSI_PINSOURCE, CC2500_SPI_AF); //MOSI

  // NSS
  GPIO_InitStructure.GPIO_Pin = CC2500_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(CC2500_NSS_PORT, &GPIO_InitStructure);
  GPIO_SetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

  SPI_I2S_DeInit(CC2500_SPI_INSTANCE);
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
  SPI_Init(CC2500_SPI_INSTANCE, &SPI_InitStructure);
  SPI_Cmd(CC2500_SPI_INSTANCE, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(CC2500_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
    ;

  SPI_I2S_ReceiveData(CC2500_SPI_INSTANCE);
}

uint8_t cc2500_spi_transfer_byte(uint8_t data) {
  for (uint16_t spiTimeout = 0x1000; SPI_I2S_GetFlagStatus(CC2500_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET;) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      return 0;
    }
  }

  // Send out data
  SPI_I2S_SendData(CC2500_SPI_INSTANCE, data);

  //wait to receive something ... timeout if nothing comes in
  for (uint16_t spiTimeout = 0x1000; SPI_I2S_GetFlagStatus(CC2500_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET;) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      return 0;
    }
  }

  // Return back received data in SPIx->DR
  return SPI_I2S_ReceiveData(CC2500_SPI_INSTANCE);
}

void cc2500_strobe(uint8_t address) {
  cc2500_csn_enable();
  cc2500_spi_transfer_byte(address);
  cc2500_csn_disable();
}

uint8_t cc2500_read_reg(uint8_t reg) {
  cc2500_csn_enable();
  cc2500_spi_transfer_byte(reg | 0x80);
  const uint32_t ret = cc2500_spi_transfer_byte(0x00);
  cc2500_csn_disable();
  return ret;
}

uint8_t cc2500_write_reg(uint8_t reg, uint8_t data) {
  cc2500_csn_enable();
  cc2500_spi_transfer_byte(reg);
  const uint32_t ret = cc2500_spi_transfer_byte(data);
  cc2500_csn_disable();
  return ret;
}

void cc2500_reset(void) {
  cc2500_strobe(CC2500_SRES);
  delay(1000); // 1000us
  cc2500_strobe(CC2500_SIDLE);
}

void cc2500_init(void) {
  cc2500_hardware_init();
  cc2500_reset();
}

#endif