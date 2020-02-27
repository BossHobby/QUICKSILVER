#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"

#define FILES_SECTOR_OFFSET bounds.sector_size

data_flash_header_t data_flash_header;
static data_flash_bounds_t bounds;

static data_flash_file_t *current_file() {
  return &data_flash_header.files[data_flash_header.file_num - 1];
}

cbor_result_t data_flash_read_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));
  return CBOR_OK;
#endif
}

cbor_result_t data_flash_write_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x0, (uint8_t *)h, sizeof(data_flash_header_t));

  return CBOR_OK;
#endif
}

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
  m25p16_get_bounds(&bounds);

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_read_header(&data_flash_header);
  if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
    data_flash_header.file_num = 0;
  }
#endif
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.file_num = 0;
  data_flash_write_header(&data_flash_header);
  m25p16_wait_for_ready();
  reset_looptime();
#endif
}

void data_flash_restart() {
#ifdef USE_M25P16
  uint32_t offset = 0;

  for (uint16_t i = 0; i < data_flash_header.file_num; i++) {
    const uint32_t size = data_flash_header.files[i].entries * 0x80;

    offset += size / bounds.sector_size;
    if (size % bounds.sector_size > 0) {
      offset += 1;
    }
  }

  data_flash_header.files[data_flash_header.file_num].start_sector = offset;
  data_flash_header.file_num++;

  data_flash_write_header(&data_flash_header);
  reset_looptime();
#endif
}

void data_flash_finish() {
  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

cbor_result_t data_flash_read_backbox(const uint32_t index, blackbox_t *b) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();

  const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + index * 0x80;
  m25p16_read_addr(M25P16_READ_DATA_BYTES, offset, (uint8_t *)b, sizeof(blackbox_t));

  return CBOR_OK;
#endif
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);

  const uint32_t offset = FILES_SECTOR_OFFSET + current_file()->start_sector * bounds.sector_size + current_file()->entries * 0x80;
  m25p16_write_addr(M25P16_PAGE_PROGRAM, offset, (uint8_t *)b, sizeof(blackbox_t));

  current_file()->entries += 1;

  return CBOR_OK;
#endif
}