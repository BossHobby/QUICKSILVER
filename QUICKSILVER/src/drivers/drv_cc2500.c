#include "drv_cc2500.h"

#include "drv_time.h"
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
#define CC2500_GDO0_PINSOURCE GPIO_PinSource14
#define CC2500_GDO0_PIN GPIO_Pin_14
#define CC2500_GDO0_PORT GPIOC
#endif

#ifdef USE_CC2500_PA_LNA

#ifdef CC2500_TX_EN_PA8
#define CC2500_TX_EN_PINSOURCE GPIO_PinSource8
#define CC2500_TX_EN_PIN GPIO_Pin_8
#define CC2500_TX_EN_PORT GPIOA
#endif

#ifdef CC2500_LNA_EN_PA13
#define CC2500_LNA_EN_PINSOURCE GPIO_PinSource13
#define CC2500_LNA_EN_PIN GPIO_Pin_13
#define CC2500_LNA_EN_PORT GPIOA
#endif

#if defined(USE_CC2500_DIVERSITY)
#ifdef CC2500_ANT_SEL_PA14
#define CC2500_ANT_SEL_PINSOURCE GPIO_PinSource14
#define CC2500_ANT_SEL_PIN GPIO_Pin_14
#define CC2500_ANT_SEL_PORT GPIOA
#endif
#endif

#endif

extern int liberror;

void cc2500_csn_enable() {
  GPIO_ResetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);
}

void cc2500_csn_disable() {
  GPIO_SetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);
}

uint8_t cc2500_read_gdo0() {
  return GPIO_ReadInputDataBit(CC2500_GDO0_PORT, CC2500_GDO0_PIN);
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
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(CC2500_NSS_PORT, &GPIO_InitStructure);
  GPIO_SetBits(CC2500_NSS_PORT, CC2500_NSS_PIN);

#if defined(USE_CC2500_PA_LNA)
  GPIO_InitStructure.GPIO_Pin = CC2500_LNA_EN_PIN;
  GPIO_Init(CC2500_LNA_EN_PORT, &GPIO_InitStructure);
  // turn antenna on
  GPIO_SetBits(CC2500_LNA_EN_PORT, CC2500_LNA_EN_PIN);

  GPIO_InitStructure.GPIO_Pin = CC2500_TX_EN_PIN;
  GPIO_Init(CC2500_TX_EN_PORT, &GPIO_InitStructure);
  // turn tx off
  GPIO_ResetBits(CC2500_LNA_EN_PORT, CC2500_TX_EN_PIN);

#if defined(USE_CC2500_DIVERSITY)
  GPIO_InitStructure.GPIO_Pin = CC2500_ANT_SEL_PIN;
  GPIO_Init(CC2500_ANT_SEL_PORT, &GPIO_InitStructure);
  // choose b?
  GPIO_SetBits(CC2500_LNA_EN_PORT, CC2500_ANT_SEL_PIN);
#endif

#endif // USE_CC2500_PA_LNA

  // GDO0
  GPIO_InitStructure.GPIO_Pin = CC2500_GDO0_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = 0;
  GPIO_InitStructure.GPIO_OType = 0;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(CC2500_GDO0_PORT, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

  SPI_I2S_DeInit(CC2500_SPI_INSTANCE);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
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
  for (uint16_t spiTimeout = 1000; SPI_I2S_GetFlagStatus(CC2500_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET;) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      return 0;
    }
  }

  // Send out data
  SPI_I2S_SendData(CC2500_SPI_INSTANCE, data);

  //wait to receive something ... timeout if nothing comes in
  for (uint16_t spiTimeout = 1000; SPI_I2S_GetFlagStatus(CC2500_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET;) {
    if ((spiTimeout--) == 0) {
      liberror++; //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      return 0;
    }
  }

  // Return back received data in SPIx->DR
  return SPI_I2S_ReceiveData(CC2500_SPI_INSTANCE);
}

inline void cc2500_strobe(uint8_t address) {
  cc2500_csn_enable();
  cc2500_spi_transfer_byte(address);
  cc2500_csn_disable();
}

inline uint8_t cc2500_get_status(void) {
  cc2500_csn_enable();
  uint8_t status = cc2500_spi_transfer_byte(0xFF);
  cc2500_csn_disable();
  return status;
}

inline uint8_t cc2500_read_reg(uint8_t reg) {
  cc2500_csn_enable();
  cc2500_spi_transfer_byte(reg | CC2500_READ_SINGLE);
  const uint32_t ret = cc2500_spi_transfer_byte(0xFF);
  cc2500_csn_disable();
  return ret;
}

uint8_t cc2500_read_multi(uint8_t reg, uint8_t data, uint8_t *result, uint8_t len) {
  cc2500_csn_enable();
  const uint8_t ret = cc2500_spi_transfer_byte(reg);
  for (uint8_t i = 0; i < len; i++) {
    result[i] = cc2500_spi_transfer_byte(data);
  }
  cc2500_csn_disable();
  return ret;
}

inline uint8_t cc2500_read_fifo(uint8_t *result, uint8_t len) {
  return cc2500_read_multi(CC2500_FIFO | CC2500_READ_BURST, 0xFF, result, len);
}

uint8_t cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  cc2500_csn_enable();
  const uint8_t ret = cc2500_spi_transfer_byte(reg);
  for (uint8_t i = 0; i < len; i++) {
    cc2500_spi_transfer_byte(data[i]);
  }
  cc2500_csn_disable();
  return ret;
}

inline uint8_t cc2500_write_fifo(uint8_t *data, uint8_t len) {
  // flush tx fifo
  cc2500_strobe(CC2500_SFTX);

  const uint8_t ret = cc2500_write_multi(CC2500_FIFO | CC2500_WRITE_BURST, data, len);

  // and send!
  cc2500_strobe(CC2500_STX);

  // while ((cc2500_get_status() & (0x70)) != (1 << 4))
  //   ;

  return ret;
}

inline uint8_t cc2500_write_reg(uint8_t reg, uint8_t data) {
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

void cc2500_switch_antenna() {
#if defined(USE_CC2500_PA_LNA) && defined(USE_CC2500_DIVERSITY)
  static uint8_t alternative_selected = 1;
  if (alternative_selected == 1) {
    GPIO_ResetBits(CC2500_ANT_SEL_PORT, CC2500_ANT_SEL_PIN);
  } else {
    GPIO_SetBits(CC2500_ANT_SEL_PORT, CC2500_ANT_SEL_PIN);
  }
  alternative_selected = alternative_selected ? 0 : 1;
#endif
}

void cc2500_enter_rxmode(void) {
#if defined(USE_CC2500_PA_LNA)
  GPIO_SetBits(CC2500_LNA_EN_PORT, CC2500_LNA_EN_PIN);
  GPIO_ResetBits(CC2500_TX_EN_PORT, CC2500_TX_EN_PIN);
#endif
}

void cc2500_enter_txmode(void) {
#if defined(USE_CC2500_PA_LNA)
  GPIO_ResetBits(CC2500_LNA_EN_PORT, CC2500_LNA_EN_PIN);
  GPIO_SetBits(CC2500_TX_EN_PORT, CC2500_TX_EN_PIN);
#endif
}

void cc2500_set_power(uint8_t power) {
  static const uint8_t patable[8] = {
      0xC5, // -12dbm
      0x97, // -10dbm
      0x6E, // -8dbm
      0x7F, // -6dbm
      0xA9, // -4dbm
      0xBB, // -2dbm
      0xFE, // 0dbm
      0xFF  // 1.5dbm
  };

  if (power > 7)
    power = 7;

  cc2500_write_reg(CC2500_PATABLE, patable[power]);
}

#endif