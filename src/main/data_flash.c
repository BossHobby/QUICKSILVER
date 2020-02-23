#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_time.h"
#include "usb_configurator.h"
#include "util.h"

data_flash_header_t data_flash_header;

static data_flash_bounds_t bounds;
static uint8_t buffer[255];

cbor_result_t data_flash_read_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  memset(buffer, 0, 255);

  m25p16_wait_for_ready();
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x0, buffer, 255);

  cbor_value_t dec;
  cbor_result_t res = CBOR_OK;
  cbor_decoder_init(&dec, buffer, 255);

  res = cbor_decode_uint32(&dec, &h->entries);
  if (res < CBOR_OK) {
    return res;
  }

  return CBOR_OK;
#endif
}

cbor_result_t data_flash_write_header(data_flash_header_t *h) {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_SECTOR_ERASE, 0x0, NULL, 0);

  memset(buffer, 0, 255);

  cbor_value_t enc;
  cbor_result_t res = CBOR_OK;
  cbor_encoder_init(&enc, buffer, 255);

  res = cbor_encode_uint32(&enc, &h->entries);
  if (res < CBOR_OK) {
    return res;
  }

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x0, buffer, cbor_encoder_len(&enc));

  return CBOR_OK;
#endif
}

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
  m25p16_get_bounds(&bounds);

  data_flash_header.entries = 0;
  data_flash_read_header(&data_flash_header);
#endif
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);
  m25p16_wait_for_ready();
  reset_looptime();
#endif
}

void data_flash_restart() {
#ifdef USE_M25P16
  const uint32_t next_sector = (data_flash_header.entries / 0x10000 + 1) * 0x10000;

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_SECTOR_ERASE, next_sector, NULL, 0);

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
  memset(buffer, 0, 255);

  m25p16_wait_for_ready();
  m25p16_read_addr(M25P16_READ_DATA_BYTES, 0x10000 + index * 0x100, buffer, 255);

  cbor_value_t dec;
  cbor_decoder_init(&dec, buffer, 255);
  return cbor_decode_compact_blackbox_t(&dec, b);
#endif
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
#ifdef USE_M25P16
  memset(buffer, 0, 255);

  cbor_value_t enc;
  cbor_result_t res;
  cbor_encoder_init(&enc, buffer, 255);

  res = cbor_encode_compact_blackbox_t(&enc, b);
  if (res < CBOR_OK) {
    return res;
  }

  quic_debugf("BLACKBOX SIZE %d", cbor_encoder_len(&enc));

  m25p16_wait_for_ready();
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, 0x10000 + data_flash_header.entries * 0x100, buffer, 255);

  data_flash_header.entries += 1;

  return CBOR_OK;
#endif
}