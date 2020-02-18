#include "data_flash.h"

#include <string.h>

#include "drv_spi_m25p16.h"
#include "drv_time.h"

data_flash_header_t data_flash_header;

void data_flash_init() {
#ifdef USE_M25P16
  m25p16_init();
#endif
}

void data_flash_reset() {
#ifdef USE_M25P16
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_command(M25P16_BULK_ERASE);

  data_flash_header.time = timer_millis();
  data_flash_header.entries = 0;
#endif
}

cbor_result_t data_flash_read_backbox(const uint32_t index, blackbox_t *b) {
#ifdef USE_M25P16
  while ((m25p16_command(M25P16_READ_STATUS_REGISTER) & 0x01))
    ;

  uint8_t data[255];
  m25p16_read_addr(M25P16_READ_DATA_BYTES, index * 0x100, data, 255);

  cbor_value_t dec;
  cbor_decoder_init(&dec, data, 255);
  return cbor_decode_compact_blackbox_t(&dec, b);
#endif
}

cbor_result_t data_flash_write_backbox(const blackbox_t *b) {
#ifdef USE_M25P16
  static uint8_t data[255];
  memset(data, 0, 255);

  cbor_value_t enc;
  cbor_encoder_init(&enc, data, 255);

  cbor_result_t res = cbor_encode_compact_blackbox_t(&enc, b);
  if (res < CBOR_OK) {
    return res;
  }

  while ((m25p16_command(M25P16_READ_STATUS_REGISTER) & 0x01) != 0)
    ;
  m25p16_command(M25P16_WRITE_ENABLE);
  m25p16_write_addr(M25P16_PAGE_PROGRAM, data_flash_header.entries * 0x100, data, 255);
  data_flash_header.entries += 1;

  return res;
#endif
}