#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"

data_flash_header_t data_flash_header;
static data_flash_bounds_t bounds;

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
  data_flash_header.entries = 0;
  data_flash_read_header(&data_flash_header);
  if (data_flash_header.magic != DATA_FLASH_HEADER_MAGIC) {
    data_flash_header.entries = 0;
  }
#endif
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  data_flash_header.magic = DATA_FLASH_HEADER_MAGIC;
  data_flash_header.entries = 0;
  data_flash_write_header(&data_flash_header);
  m25p16_wait_for_ready();
  reset_looptime();
#endif
}

void data_flash_restart() {
#ifdef USE_M25P16
  //data_flash_write_header(&data_flash_header);
  //reset_looptime();
#endif
}

void data_flash_finish() {
  data_flash_write_header(&data_flash_header);
  reset_looptime();
}

cbor_result_t data_flash_read_backbox(const uint32_t index, blackbox_t *b) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x100000 + index * 0x100, (uint8_t *)b, sizeof(blackbox_t));
  return CBOR_OK;
#endif
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x100000 + data_flash_header.entries * 0x100, (uint8_t *)b, sizeof(blackbox_t));

  data_flash_header.entries += 1;

  return CBOR_OK;
#endif
}