#include "drv_spi_m25p16.h"

#include "drv_spi.h"
#include "project.h"

#if defined(F4) && defined(USE_M25P16)

#define SPI_PORT spi_port_defs[M25P16_SPI_PORT]
#define SCK_PIN gpio_pin_defs[SPI_PORT.sck]
#define MISO_PIN gpio_pin_defs[SPI_PORT.miso]
#define MOSI_PIN gpio_pin_defs[SPI_PORT.mosi]
#define NSS_PIN gpio_pin_defs[M25P16_NSS_PIN]

#define JEDEC_ID_MACRONIX_MX25L3206E 0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E 0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E 0xC22019
#define JEDEC_ID_MICRON_M25P16 0x202015
#define JEDEC_ID_MICRON_N25Q064 0x20BA17
#define JEDEC_ID_MICRON_N25Q128 0x20ba18
#define JEDEC_ID_WINBOND_W25Q16 0xEF4015
#define JEDEC_ID_WINBOND_W25Q32 0xEF4016
#define JEDEC_ID_WINBOND_W25Q64 0xEF4017
#define JEDEC_ID_WINBOND_W25Q128 0xEF4018
#define JEDEC_ID_WINBOND_W25Q128_DTR 0xEF7018
#define JEDEC_ID_WINBOND_W25Q256 0xEF4019
#define JEDEC_ID_CYPRESS_S25FL128L 0x016018
#define JEDEC_ID_BERGMICRO_W25Q32 0xE04016

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

  GPIO_InitStructure.GPIO_Pin = MOSI_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MOSI_PIN.port, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = MISO_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(MISO_PIN.port, &GPIO_InitStructure);

  GPIO_PinAFConfig(SCK_PIN.port, SCK_PIN.pin_source, SPI_PORT.gpio_af);
  GPIO_PinAFConfig(MISO_PIN.port, MISO_PIN.pin_source, SPI_PORT.gpio_af);
  GPIO_PinAFConfig(MOSI_PIN.port, MOSI_PIN.pin_source, SPI_PORT.gpio_af);

  GPIO_InitStructure.GPIO_Pin = NSS_PIN.pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(NSS_PIN.port, &GPIO_InitStructure);

  m25p16_csn_disable();

  SPI_I2S_DeInit(SPI_PORT.channel);
  spi_enable_rcc(M25P16_SPI_PORT);

  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(SPI_PORT.channel, &SPI_InitStructure);

  SPI_Cmd(SPI_PORT.channel, ENABLE);
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
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0x0);
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
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0x0);
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

uint8_t m25p16_read_status() {
  uint8_t status = 0;
  m25p16_read_command(M25P16_READ_STATUS_REGISTER, &status, 1);
  return status;
}

void m25p16_wait_for_ready() {
  while ((m25p16_read_status() & 0x01) != 0)
    ;
}

void m25p16_get_bounds(data_flash_bounds_t *bounds) {
  m25p16_wait_for_ready();
  uint8_t raw_id[3];
  m25p16_read_command(M25P16_READ_IDENTIFICATION, raw_id, 3);

  const uint32_t chip_id = (raw_id[0] << 16) | (raw_id[1] << 8) | raw_id[2] << 16;
  switch (chip_id) {
  case JEDEC_ID_WINBOND_W25Q16:
  case JEDEC_ID_MICRON_M25P16:
    bounds->sectors = 32;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_BERGMICRO_W25Q32:
    bounds->sectors = 1024;
    bounds->pages_per_sector = 16;
    break;
  case JEDEC_ID_WINBOND_W25Q32:
  case JEDEC_ID_MACRONIX_MX25L3206E:
    bounds->sectors = 64;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q064:
  case JEDEC_ID_WINBOND_W25Q64:
  case JEDEC_ID_MACRONIX_MX25L6406E:
    bounds->sectors = 128;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q128:
  case JEDEC_ID_WINBOND_W25Q128:
  case JEDEC_ID_WINBOND_W25Q128_DTR:
  case JEDEC_ID_CYPRESS_S25FL128L:
    bounds->sectors = 256;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_WINBOND_W25Q256:
  case JEDEC_ID_MACRONIX_MX25L25635E:
    bounds->sectors = 512;
    bounds->pages_per_sector = 256;
    break;
  default:
    // Unsupported chip or not an SPI NOR flash
    bounds->sectors = 0;
    bounds->pages_per_sector = 0;
    bounds->sector_size = 0;
    bounds->total_size = 0;
    return;
  }

  bounds->page_size = 256;
  bounds->sector_size = bounds->pages_per_sector * bounds->page_size;
  bounds->total_size = bounds->sector_size * bounds->sectors;
}

#endif