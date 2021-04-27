#include "drv_spi_m25p16.h"

#include <stm32f4xx_ll_spi.h>
#include <string.h>

#include "drv_spi.h"
#include "project.h"

#if defined(STM32F4) && defined(USE_M25P16)

#define SPI_PORT spi_port_defs[M25P16_SPI_PORT]
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

static void m25p16_reinit() {
  LL_SPI_Disable(SPI_PORT.channel);

  LL_SPI_DeInit(SPI_PORT.channel);
  LL_SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStructure.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStructure.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStructure.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStructure.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStructure.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStructure.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStructure.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStructure.CRCPoly = 7;
  LL_SPI_Init(SPI_PORT.channel, &SPI_InitStructure);
  LL_SPI_Enable(SPI_PORT.channel);
}

void m25p16_init() {
  spi_init_pins(M25P16_SPI_PORT, M25P16_NSS_PIN);

  spi_enable_rcc(M25P16_SPI_PORT);

  m25p16_reinit();

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(SPI_PORT.channel) == RESET)
    ;

  LL_SPI_ReceiveData8(SPI_PORT.channel);

  spi_dma_init(M25P16_SPI_PORT);
}

uint8_t m25p16_is_ready() {
  if (!spi_dma_is_ready(M25P16_SPI_PORT)) {
    return 0;
  }

  m25p16_reinit();

  spi_csn_enable(M25P16_NSS_PIN);
  spi_transfer_byte(M25P16_SPI_PORT, M25P16_READ_STATUS_REGISTER);
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, 0x0);
  spi_csn_disable(M25P16_NSS_PIN);

  return (ret & 0x01) == 0;
}

void m25p16_wait_for_ready() {
  while (!m25p16_is_ready())
    ;
}

uint8_t m25p16_command(const uint8_t cmd) {
  m25p16_wait_for_ready();

  spi_csn_enable(M25P16_NSS_PIN);
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  spi_csn_disable(M25P16_NSS_PIN);
  return ret;
}

uint8_t m25p16_read_command(const uint8_t cmd, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  spi_csn_enable(M25P16_NSS_PIN);
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0x0);
  }
  spi_csn_disable(M25P16_NSS_PIN);
  return ret;
}

static void m25p16_set_addr(const uint32_t addr) {
  spi_transfer_byte(M25P16_SPI_PORT, (addr >> 16) & 0xFF);
  spi_transfer_byte(M25P16_SPI_PORT, (addr >> 8) & 0xFF);
  spi_transfer_byte(M25P16_SPI_PORT, addr & 0xFF);
}

uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  spi_csn_enable(M25P16_NSS_PIN);
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  m25p16_set_addr(addr);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_transfer_byte(M25P16_SPI_PORT, 0x0);
  }
  spi_csn_disable(M25P16_NSS_PIN);
  return ret;
}

uint8_t m25p16_page_program(const uint32_t addr, const uint8_t *buf, const uint32_t size) {
  if (!spi_dma_is_ready(M25P16_SPI_PORT)) {
    return 0;
  }

  m25p16_reinit();

  spi_csn_enable(M25P16_NSS_PIN);
  spi_transfer_byte(M25P16_SPI_PORT, M25P16_WRITE_ENABLE);
  spi_csn_disable(M25P16_NSS_PIN);

  spi_csn_enable(M25P16_NSS_PIN);

  static uint8_t dma_buf[256];
  dma_buf[0] = M25P16_PAGE_PROGRAM;
  dma_buf[1] = (addr >> 16) & 0xFF;
  dma_buf[2] = (addr >> 8) & 0xFF;
  dma_buf[3] = addr & 0xFF;
  memcpy(dma_buf + 4, buf, size);

  spi_dma_transfer_begin(M25P16_SPI_PORT, dma_buf, size + 4);
  return 1;
}

void m25p16_dma_rx_isr() {
  spi_csn_disable(M25P16_NSS_PIN);
}

uint8_t m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_command(M25P16_WRITE_ENABLE);

  spi_csn_enable(M25P16_NSS_PIN);
  const uint8_t ret = spi_transfer_byte(M25P16_SPI_PORT, cmd);
  m25p16_set_addr(addr);
  for (uint8_t i = 0; i < len; i++) {
    spi_transfer_byte(M25P16_SPI_PORT, data[i]);
  }
  spi_csn_disable(M25P16_NSS_PIN);
  return ret;
}

void m25p16_get_bounds(data_flash_bounds_t *bounds) {
  uint8_t raw_id[3];
  m25p16_read_command(M25P16_READ_IDENTIFICATION, raw_id, 3);

  const uint32_t chip_id = (raw_id[0] << 16) | (raw_id[1] << 8) | raw_id[2];
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

  bounds->page_size = M25P16_PAGE_SIZE;
  bounds->sector_size = bounds->pages_per_sector * bounds->page_size;
  bounds->total_size = bounds->sector_size * bounds->sectors;
}

#endif