#include "drv_spi_m25p16.h"

#include "drv_spi.h"
#include "project.h"

#if defined(F4) && defined(USE_M25P16)

#define SPI_PORT spi_port_defs[M25P16_SPI_PORT]
#define SCK_PIN gpio_pin_defs[SPI_PORT.sck]
#define MISO_PIN gpio_pin_defs[SPI_PORT.miso]
#define MOSI_PIN gpio_pin_defs[SPI_PORT.mosi]
#define NSS_PIN gpio_pin_defs[M25P16_NSS_PIN]

void m25p16_csn_enable() {
  GPIO_ResetBits(NSS_PIN.port, NSS_PIN.pin);
}

void m25p16_csn_disable() {
  GPIO_SetBits(NSS_PIN.port, NSS_PIN.pin);
}

void m25p16_init() {
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SCK_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SCK_PIN.port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MISO_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MISO_PIN.port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MOSI_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(MOSI_PIN.port, &GPIO_InitStructure);

  GPIO_PinAFConfig(SCK_PIN.port, SCK_PIN.pin_source, SPI_PORT.gpio_af);
  GPIO_PinAFConfig(MISO_PIN.port, MISO_PIN.pin_source, SPI_PORT.gpio_af);
  GPIO_PinAFConfig(MOSI_PIN.port, MOSI_PIN.pin_source, SPI_PORT.gpio_af);

  GPIO_InitStructure.GPIO_Pin = NSS_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(NSS_PIN.port, &GPIO_InitStructure);

  m25p16_csn_disable();

  spi_enable_rcc(M25P16_SPI_PORT);

  SPI_I2S_DeInit(SPI_PORT.channel);
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
  SPI_Init(SPI_PORT.channel, &SPI_InitStructure);
  SPI_Cmd(SPI_PORT.channel, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(SPI_PORT.channel, SPI_I2S_FLAG_TXE) == RESET)
    ;

  SPI_I2S_ReceiveData(SPI_PORT.channel);
}

uint8_t m25p16_command(const uint8_t cmd) {
  m25p16_csn_enable();
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  m25p16_csn_disable();
  return ret;
}

uint8_t m25p16_read_command(const uint8_t cmd, uint8_t *data, const uint32_t len) {
  m25p16_csn_enable();
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0xFF);
  }
  m25p16_csn_disable();
  return ret;
}

static void m25p16_set_addr(const uint32_t addr) {
  spi_transfer_byte(M25P16_SPI_PORT, (addr >> 16) & 0xFF);
  spi_transfer_byte(M25P16_SPI_PORT, (addr >> 8) & 0xFF);
  spi_transfer_byte(M25P16_SPI_PORT, addr & 0xFF);
}

uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_csn_enable();
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  m25p16_set_addr(addr);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0xFF);
  }
  m25p16_csn_disable();
  return ret;
}

uint8_t m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_csn_enable();
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  m25p16_set_addr(addr);
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer_byte(M25P16_SPI_PORT, data[i]);
  }
  m25p16_csn_disable();
  return ret;
}

#endif