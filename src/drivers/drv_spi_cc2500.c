#include "drv_spi_cc2500.h"

#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"

#if defined(F4) && defined(USE_CC2500)

#define PORT spi_port_defs[CC2500_SPI_PORT]

#ifdef CC2500_GDO0_PC14
#define CC2500_GDO0_PINSOURCE GPIO_PinSource14
#define CC2500_GDO0_PIN GPIO_Pin_14
#define CC2500_GDO0_PORT GPIOC
#endif

#ifdef CC2500_GDO0_PB6
#define CC2500_GDO0_PINSOURCE GPIO_PinSource6
#define CC2500_GDO0_PIN GPIO_Pin_6
#define CC2500_GDO0_PORT GPIOB
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

uint8_t cc2500_read_gdo0() {
  return GPIO_ReadInputDataBit(CC2500_GDO0_PORT, CC2500_GDO0_PIN);
}

static void cc2500_hardware_init() {
  spi_init_pins(CC2500_SPI_PORT, CC2500_NSS);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

#if defined(USE_CC2500_PA_LNA)
  GPIO_InitStructure.GPIO_Pin = CC2500_LNA_EN_PIN;
  GPIO_Init(CC2500_LNA_EN_PORT, &GPIO_InitStructure);
  // turn antenna on
  GPIO_SetBits(CC2500_LNA_EN_PORT, CC2500_LNA_EN_PIN);

  GPIO_InitStructure.GPIO_Pin = CC2500_TX_EN_PIN;
  GPIO_Init(CC2500_TX_EN_PORT, &GPIO_InitStructure);
  // turn tx off
  GPIO_ResetBits(CC2500_TX_EN_PORT, CC2500_TX_EN_PIN);

#if defined(USE_CC2500_DIVERSITY)
  GPIO_InitStructure.GPIO_Pin = CC2500_ANT_SEL_PIN;
  GPIO_Init(CC2500_ANT_SEL_PORT, &GPIO_InitStructure);
  // choose b?
  GPIO_SetBits(CC2500_ANT_SEL_PORT, CC2500_ANT_SEL_PIN);
#endif

#endif // USE_CC2500_PA_LNA

  // GDO0
  GPIO_InitStructure.GPIO_Pin = CC2500_GDO0_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(CC2500_GDO0_PORT, &GPIO_InitStructure);

  spi_enable_rcc(CC2500_SPI_PORT);

  SPI_I2S_DeInit(PORT.channel);
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
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(PORT.channel);

  spi_dma_init(CC2500_SPI_PORT);
}

void cc2500_strobe(uint8_t address) {
  spi_csn_enable(CC2500_NSS);
  spi_transfer_byte(CC2500_SPI_PORT, address);
  spi_csn_disable(CC2500_NSS);
}

uint8_t cc2500_get_status() {
  spi_csn_enable(CC2500_NSS);
  const uint8_t status = spi_transfer_byte(CC2500_SPI_PORT, 0xFF);
  spi_csn_disable(CC2500_NSS);
  return status;
}

uint8_t cc2500_write_reg(uint8_t reg, uint8_t data) {
  spi_csn_enable(CC2500_NSS);
  spi_transfer_byte(CC2500_SPI_PORT, reg | CC2500_WRITE_SINGLE);
  const uint8_t ret = spi_transfer_byte(CC2500_SPI_PORT, data);
  spi_csn_disable(CC2500_NSS);
  return ret;
}

uint8_t cc2500_read_reg(uint8_t reg) {
  return cc2500_write_reg(reg | CC2500_READ_SINGLE, 0xFF);
}

static uint8_t cc2500_read_multi(uint8_t reg, uint8_t data, uint8_t *result, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  uint8_t buffer[len + 1];
  buffer[0] = reg;
  for (uint8_t i = 0; i < len; i++) {
    buffer[i + 1] = data;
  }

  spi_dma_transfer_bytes(CC2500_SPI_PORT, buffer, len + 1);
  for (uint8_t i = 0; i < len; i++) {
    result[i] = buffer[i + 1];
  }

  spi_csn_disable(CC2500_NSS);
  return buffer[0];
}

/*
static uint8_t cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  uint8_t buffer[len + 1];
  buffer[0] = reg;
  for (uint8_t i = 0; i < len; i++) {
    buffer[i + 1] = data[i];
  }

  spi_dma_transfer_bytes(CC2500_SPI_PORT, buffer, len + 1);
  spi_csn_disable(CC2500_NSS);
  return buffer[0];
}
*/

static uint8_t cc2500_write_multi(uint8_t reg, uint8_t *data, uint8_t len) {
  spi_csn_enable(CC2500_NSS);

  const uint8_t ret = spi_transfer_byte(CC2500_SPI_PORT, reg);
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer_byte(CC2500_SPI_PORT, data[i]);
  }

  spi_csn_disable(CC2500_NSS);
  return ret;
}

uint8_t cc2500_read_fifo(uint8_t *result, uint8_t len) {
  return cc2500_read_multi(CC2500_FIFO | CC2500_READ_BURST, 0xFF, result, len);
}

uint8_t cc2500_write_fifo(uint8_t *data, uint8_t len) {
  // flush tx fifo
  cc2500_strobe(CC2500_SFTX);

  const uint8_t ret = cc2500_write_multi(CC2500_FIFO | CC2500_WRITE_BURST, data, len);

  // and send!
  cc2500_strobe(CC2500_STX);

  return ret;
}

void cc2500_reset() {
  cc2500_strobe(CC2500_SRES);
  delay(1000); // 1000us
  cc2500_strobe(CC2500_SIDLE);
}

void cc2500_init() {
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

void cc2500_enter_rxmode() {
#if defined(USE_CC2500_PA_LNA)
  GPIO_SetBits(CC2500_LNA_EN_PORT, CC2500_LNA_EN_PIN);
  GPIO_ResetBits(CC2500_TX_EN_PORT, CC2500_TX_EN_PIN);
#endif
}

void cc2500_enter_txmode() {
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