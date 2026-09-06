#include "driver/blackbox/m25p16.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi.h"
#include "util/util.h"

#define M25P16_BAUD_RATE MHZ_TO_HZ(21)

#define JEDEC_ID_MACRONIX_MX25L3206E 0xC22016
#define JEDEC_ID_MACRONIX_MX25L6406E 0xC22017
#define JEDEC_ID_MACRONIX_MX25L25635E 0xC22019
#define JEDEC_ID_MICRON_M25P16 0x202015
#define JEDEC_ID_MICRON_N25Q064 0x20BA17
#define JEDEC_ID_MICRON_N25Q128 0x20ba18
#define JEDEC_ID_WINBOND_W25Q80 0xEF4014
#define JEDEC_ID_WINBOND_W25Q16 0xEF4015
#define JEDEC_ID_WINBOND_W25X32 0xEF3016
#define JEDEC_ID_WINBOND_W25Q32 0xEF4016
#define JEDEC_ID_WINBOND_W25Q64 0xEF4017
#define JEDEC_ID_WINBOND_W25Q64_DTR 0xEF7017
#define JEDEC_ID_WINBOND_W25Q128 0xEF4018
#define JEDEC_ID_WINBOND_W25Q128_DTR 0xEF7018
#define JEDEC_ID_WINBOND_W25Q256 0xEF4019
#define JEDEC_ID_CYPRESS_S25FL064L 0x016017
#define JEDEC_ID_CYPRESS_S25FL128L 0x016018
#define JEDEC_ID_PUYA_PY25Q128 0x852018
#define JEDEC_ID_ZBIT_ZB25VQ128 0x5E4018
#define JEDEC_ID_BERGMICRO_W25Q32 0xE04016

#ifdef USE_DATA_FLASH

static spi_bus_device_t bus = {};

static uint8_t m25p16_addr_size() {
  return blackbox_bounds.use_4byte_addresses ? 4 : 3;
}

static void m25p16_make_cmd_addr(uint8_t *buffer, const uint8_t cmd, const uint32_t addr) {
  buffer[0] = cmd;
  if (blackbox_bounds.use_4byte_addresses) {
    buffer[1] = (addr >> 24) & 0xFF;
    buffer[2] = (addr >> 16) & 0xFF;
    buffer[3] = (addr >> 8) & 0xFF;
    buffer[4] = addr & 0xFF;
    return;
  }

  buffer[1] = (addr >> 16) & 0xFF;
  buffer[2] = (addr >> 8) & 0xFF;
  buffer[3] = addr & 0xFF;
}

void m25p16_init() {
  if (!target_spi_device_valid(&target.flash)) {
    return;
  }

  bus.port = target.flash.port;
  bus.nss = target.flash.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, M25P16_BAUD_RATE);
}

bool m25p16_is_ready() {
  if (!spi_txn_ready(&bus)) {
    spi_txn_continue(&bus);
    return false;
  }

  static uint8_t buffer[2];
  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 2),
  };
  const bool is_done = spi_seg_submit_check(&bus, segs, {
    buffer[0] = M25P16_READ_STATUS_REGISTER;
    buffer[1] = 0xFF;
  });
  if (!is_done) {
    return false;
  }
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

uint8_t m25p16_read_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  m25p16_wait_for_ready();

  uint8_t ret = 0;
  uint8_t cmd_addr[5];
  m25p16_make_cmd_addr(cmd_addr, cmd, addr);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, cmd_addr, 1),
      spi_make_seg_buffer(NULL, cmd_addr + 1, m25p16_addr_size()),
      spi_make_seg_buffer(data, NULL, len),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

bool m25p16_page_program(const uint32_t addr, const uint8_t *buf, const uint32_t size) {
  if (!m25p16_is_ready()) {
    return false;
  }

  uint8_t cmd_addr[5];
  m25p16_make_cmd_addr(cmd_addr, M25P16_PAGE_PROGRAM, addr);

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_WRITE_ENABLE),
    };
    spi_seg_submit(&bus, segs);
  }

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_buffer(NULL, cmd_addr, 1 + m25p16_addr_size()),
        spi_make_seg_buffer(NULL, buf, size),
    };
    spi_seg_submit(&bus, segs);
  }

  spi_txn_continue(&bus);
  return true;
}

bool m25p16_write_addr(const uint8_t cmd, const uint32_t addr, uint8_t *data, const uint32_t len) {
  if (!m25p16_is_ready()) {
    return false;
  }

  uint8_t cmd_addr[5];
  m25p16_make_cmd_addr(cmd_addr, cmd, addr);

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_WRITE_ENABLE),
    };
    spi_seg_submit(&bus, segs);
  }
  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_buffer(NULL, cmd_addr, 1 + m25p16_addr_size()),
        spi_make_seg_buffer(NULL, data, len),
    };
    spi_seg_submit(&bus, segs);
  }

  spi_txn_continue(&bus);
  return true;
}

bool m25p16_chip_erase() {
  if (!m25p16_is_ready()) {
    return false;
  }

  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_WRITE_ENABLE),
    };
    spi_seg_submit(&bus, segs);
  }
  {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(M25P16_BULK_ERASE),
    };
    spi_seg_submit(&bus, segs);
  }

  spi_txn_continue(&bus);
  return true;
}

void m25p16_get_bounds(blackbox_device_bounds_t *bounds) {
  bounds->use_4byte_addresses = false;

  uint8_t raw_id[3];
  m25p16_read_command(M25P16_READ_IDENTIFICATION, raw_id, 3);

  const uint32_t chip_id = (raw_id[0] << 16) | (raw_id[1] << 8) | raw_id[2];
  switch (chip_id) {
  case JEDEC_ID_WINBOND_W25Q80:
    bounds->sectors = 16;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_WINBOND_W25Q16:
  case JEDEC_ID_MICRON_M25P16:
    bounds->sectors = 32;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_BERGMICRO_W25Q32:
  case JEDEC_ID_WINBOND_W25X32:
  case JEDEC_ID_WINBOND_W25Q32:
  case JEDEC_ID_MACRONIX_MX25L3206E:
    bounds->sectors = 64;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q064:
  case JEDEC_ID_WINBOND_W25Q64:
  case JEDEC_ID_WINBOND_W25Q64_DTR:
  case JEDEC_ID_MACRONIX_MX25L6406E:
  case JEDEC_ID_CYPRESS_S25FL064L:
    bounds->sectors = 128;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_MICRON_N25Q128:
  case JEDEC_ID_WINBOND_W25Q128:
  case JEDEC_ID_WINBOND_W25Q128_DTR:
  case JEDEC_ID_CYPRESS_S25FL128L:
  case JEDEC_ID_PUYA_PY25Q128:
  case JEDEC_ID_ZBIT_ZB25VQ128:
    bounds->sectors = 256;
    bounds->pages_per_sector = 256;
    break;
  case JEDEC_ID_WINBOND_W25Q256:
  case JEDEC_ID_MACRONIX_MX25L25635E:
    bounds->use_4byte_addresses = true;
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

  if (bounds->use_4byte_addresses) {
    m25p16_command(M25P16_ENTER_4BYTE_ADDRESS_MODE);
  }
}

#endif
