#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_spi_sdcard.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"

data_flash_header_t data_flash_header;

static data_flash_bounds_t bounds;
static data_flash_file_t *current_file() {
  return &data_flash_header.files[data_flash_header.file_num - 1];
}

#ifdef USE_M25P16
#define FILES_SECTOR_OFFSET bounds.sector_size
#endif
#ifdef USE_SDCARD
#define FILES_SECTOR_OFFSET 1
#define ENTRIES_PER_BLOCK (512 / BLACKBOX_MAX_SIZE)
#endif

cbor_result_t data_flash_read_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));
#endif
#ifdef USE_SDCARD
  uint8_t buf[512];
  sdcard_read_sectors(buf, 0, 1);

  memcpy(h, buf, sizeof(data_flash_header_t));
#endif
  return CBOR_OK;
}

cbor_result_t data_flash_write_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));
#endif
#ifdef USE_SDCARD
  uint8_t buf[512];
  memcpy(buf, h, sizeof(data_flash_header_t));

  sdcard_write_sectors(buf, 0, 1);
#endif
  return CBOR_OK;
}

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
  m25p16_get_bounds(&bounds);
#endif
#ifdef USE_SDCARD
  sdcard_init();
  sdcard_detect();

  bounds.page_size = 512;
  bounds.pages_per_sector = 1;

  // TODO: fetch
  bounds.sectors = 512;

  bounds.sector_size = bounds.pages_per_sector * bounds.page_size;
  bounds.total_size = bounds.sector_size * bounds.sectors;
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_read_header(&data_flash_header);
  if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
    data_flash_header.file_num = 0;
  }
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);
#endif

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

void data_flash_restart() {
  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].entries * 0x80;

    offset += size / bounds.sector_size;
    if (size % bounds.sector_size > 0) {
      offset += 1;
    }
  }

  data_flash_header.files[data_flash_header.file_num].entries = 0;
  data_flash_header.files[data_flash_header.file_num].start_sector = offset;
  data_flash_header.file_num++;

  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

void data_flash_finish() {
  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

cbor_result_t data_flash_read_backbox(const uint32_t index, blackbox_t *b, const uint8_t count) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();

  for (uint32_t i = 0; i < count; i++) {
    const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + (index + i) * BLACKBOX_MAX_SIZE;
    m25p16_read_addr(M25P16_READ_DATA_BYTES, offset, (uint8_t *)b, sizeof(blackbox_t));
  }
#endif
#ifdef USE_SDCARD
  const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector + (index / ENTRIES_PER_BLOCK);
  const uint32_t sectors = count / ENTRIES_PER_BLOCK + (count % ENTRIES_PER_BLOCK ? 1 : 0);

  uint8_t buf[sectors * 512];
  sdcard_read_sectors(buf, offset, sectors);

  for (uint32_t i = 0; i < count; i++) {
    memcpy(b + i, buf + ((index % ENTRIES_PER_BLOCK) + i) * BLACKBOX_MAX_SIZE, sizeof(blackbox_t));
  }
#endif
  return CBOR_OK;
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {

#ifdef USE_M25P16
  const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + current_file()->entries * BLACKBOX_MAX_SIZE;
  if (offset >= bounds.total_size) {
    return CBOR_ERR_EOF;
  }

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);

  m25p16_write_addr(M25P16_PAGE_PROGRAM, offset, (uint8_t *)b, sizeof(blackbox_t));

#endif
#ifdef USE_SDCARD
  volatile uint32_t start = timer_micros();
  const uint32_t index = current_file()->entries % ENTRIES_PER_BLOCK;
  if (index == 0) {
    const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector + (current_file()->entries / ENTRIES_PER_BLOCK);
    sdcard_start_write_sector(offset);
  }

  sdcard_continue_write_sector(index * BLACKBOX_MAX_SIZE, b, sizeof(blackbox_t));

  if (index == ENTRIES_PER_BLOCK - 1) {
    if (!sdcard_finish_write_sector()) {
      current_file()->entries -= 3;
    }
  }
  volatile uint32_t delta = timer_micros() - start;
#endif

  current_file()->entries += 1;
  return CBOR_OK;
}