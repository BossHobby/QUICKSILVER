#include "drv_spi_m25p16.h"

#include <string.h>

#include "drv_spi.h"
#include "project.h"

#if defined(USE_M25P16)

#define M25P16_BAUD_RATE MHZ_TO_HZ(21)

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

static spi_bus_device_t bus = {
    .port = M25P16_SPI_PORT,
    .nss = M25P16_NSS_PIN,

    .auto_continue = true,
};

void m25p16_init() {
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, M25P16_BAUD_RATE);
}

uint8_t m25p16_is_ready() {
  if (!spi_txn_ready(&bus)) {
    return 0;
  }

  uint8_t buffer[2] = {M25P16_READ_STATUS_REGISTER, 0xFF};

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return (buffer[1] & 0x01) == 0;
}

void m25p16_wait_for_ready() {
  while (!m25p16_is_ready())
    ;
}

uint8_t m25p16_command(const uint8_t cmd) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &ret, &cmd, 1);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return ret;
}

uint8_t m25p16_read_command(const uint8_t cmd, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &ret, &cmd, 1);
  spi_txn_add_seg(txn, data, NULL, len);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return ret;
}

static void m25p16_set_addr(spi_txn_t *txn, const uint32_t addr) {
  spi_txn_add_seg_const(txn, (addr >> 16) & 0xFF);
  spi_txn_add_seg_const(txn, (addr >> 8) & 0xFF);
  spi_txn_add_seg_const(txn, addr & 0xFF);
}

uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &ret, &cmd, 1);
  m25p16_set_addr(txn, addr);
  spi_txn_add_seg(txn, data, NULL, len);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return ret;
}

uint8_t m25p16_page_program(const uint32_t addr, const uint8_t *buf, const uint32_t size) {
  {
    spi_txn_t *txn = spi_txn_init(&bus, NULL);
    spi_txn_add_seg_const(txn, M25P16_WRITE_ENABLE);
    spi_txn_submit(txn);
  }

  {
    spi_txn_t *txn = spi_txn_init(&bus, NULL);
    spi_txn_add_seg_const(txn, M25P16_PAGE_PROGRAM);
    m25p16_set_addr(txn, addr);
    spi_txn_add_seg(txn, NULL, buf, size);
    spi_txn_submit(txn);
  }

  spi_txn_continue(&bus);

  return 1;
}

uint8_t m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_command(M25P16_WRITE_ENABLE);

  uint8_t ret = 0;

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, &ret, &cmd, 1);
  m25p16_set_addr(txn, addr);
  spi_txn_add_seg(txn, NULL, data, len);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

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