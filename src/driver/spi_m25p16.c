#include "driver/spi_m25p16.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi.h"
#include "util/util.h"

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

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  spi_seg_submit_wait(&bus, segs);

  return (buffer[1] & 0x01) == 0;
}

void m25p16_wait_for_ready() {
  while (!m25p16_is_ready())
    ;
}

uint8_t m25p16_command(const uint8_t cmd) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, &cmd, 1),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

uint8_t m25p16_read_command(const uint8_t cmd, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, &cmd, 1),
      spi_make_seg_buffer(data, NULL, len),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

#define m25p16_set_addr(addr) \
  spi_make_seg_const((addr >> 16) & 0xFF), spi_make_seg_const((addr >> 8) & 0xFF), spi_make_seg_const(addr & 0xFF)

uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, &cmd, 1),
      m25p16_set_addr(addr),
      spi_make_seg_buffer(data, NULL, len),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

uint8_t m25p16_page_program(const uint32_t addr, const uint8_t *buf, const uint32_t size) {
  if (!m25p16_is_ready()) {
    return 0;
  }

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_WRITE_ENABLE),
    };
    spi_seg_submit(&bus, NULL, segs);
  }

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_PAGE_PROGRAM),
        m25p16_set_addr(addr),
        spi_make_seg_buffer(NULL, buf, size),
    };
    spi_seg_submit(&bus, NULL, segs);
  }

  spi_txn_continue(&bus);

  return 1;
}

uint8_t m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_command(M25P16_WRITE_ENABLE);

  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, &cmd, 1),
      m25p16_set_addr(addr),
      spi_make_seg_buffer(NULL, data, len),
  };
  spi_seg_submit_continue(&bus, NULL, segs);

  return ret;
}

void m25p16_get_bounds(blackbox_device_bounds_t *blackbox_bounds) {
  uint8_t raw_id[3];
  m25p16_read_command(M25P16_READ_IDENTIFICATION, raw_id, 3);

  const uint32_t chip_id = (raw_id[0] << 16) | (raw_id[1] << 8) | raw_id[2];
  switch (chip_id) {
  case JEDEC_ID_WINBOND_W25Q16:
  case JEDEC_ID_MICRON_M25P16:
    blackbox_bounds->sectors = 32;
    blackbox_bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_BERGMICRO_W25Q32:
    blackbox_bounds->sectors = 1024;
    blackbox_bounds->pages_per_sector = 16;
    break;
  case JEDEC_ID_WINBOND_W25Q32:
  case JEDEC_ID_MACRONIX_MX25L3206E:
    blackbox_bounds->sectors = 64;
    blackbox_bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q064:
  case JEDEC_ID_WINBOND_W25Q64:
  case JEDEC_ID_MACRONIX_MX25L6406E:
    blackbox_bounds->sectors = 128;
    blackbox_bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q128:
  case JEDEC_ID_WINBOND_W25Q128:
  case JEDEC_ID_WINBOND_W25Q128_DTR:
  case JEDEC_ID_CYPRESS_S25FL128L:
    blackbox_bounds->sectors = 256;
    blackbox_bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_WINBOND_W25Q256:
  case JEDEC_ID_MACRONIX_MX25L25635E:
    blackbox_bounds->sectors = 512;
    blackbox_bounds->pages_per_sector = 256;
    break;
  default:
    // Unsupported chip or not an SPI NOR flash
    blackbox_bounds->sectors = 0;
    blackbox_bounds->pages_per_sector = 0;
    blackbox_bounds->sector_size = 0;
    blackbox_bounds->total_size = 0;
    return;
  }

  blackbox_bounds->page_size = M25P16_PAGE_SIZE;
  blackbox_bounds->sector_size = blackbox_bounds->pages_per_sector * blackbox_bounds->page_size;
  blackbox_bounds->total_size = blackbox_bounds->sector_size * blackbox_bounds->sectors;
}

#endif