#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CBOR_OK = 0,
  CBOR_ERR_EOF = -1,
  CBOR_ERR_INVALID_TYPE = -2,
  CBOR_ERR_OVERFLOW = -3,
} cbor_error_t;

typedef enum {
  CBOR_TYPE_UINT = 0x00,
  CBOR_TYPE_NINT = 0x01,
  CBOR_TYPE_BSTR = 0x02,
  CBOR_TYPE_TSTR = 0x03,
  CBOR_TYPE_ARRAY = 0x04,
  CBOR_TYPE_MAP = 0x05,
  CBOR_TYPE_TAG = 0x06,
  CBOR_TYPE_FLOAT = 0x07,
} cbor_major_type_t;

typedef enum {
  CBOR_SIZE_BYTE = 24,
  CBOR_SIZE_SHORT = 25,
  CBOR_SIZE_WORD = 26,
  CBOR_SIZE_LONG = 27,
  CBOR_SIZE_INDEFINITE = 31,
} cbor_size_type_t;

typedef struct {
  uint8_t *start, *curr, *end;
} cbor_value_t;

typedef struct {
  cbor_value_t *dec;
  uint8_t is_streaming;
  uint32_t size;
} cbor_container_t;

typedef int32_t cbor_result_t;

void cbor_decoder_init(cbor_value_t *dec, uint8_t *data, uint32_t len);

cbor_result_t cbor_decode_type(cbor_value_t *dec);
cbor_result_t cbor_decode_flag(cbor_value_t *dec);

cbor_result_t cbor_decode_skip(cbor_value_t *dec);

cbor_result_t cbor_decode_array(cbor_value_t *dec, cbor_container_t *array);
uint32_t cbor_decode_array_size(cbor_value_t *dec, cbor_container_t *array);

cbor_result_t cbor_decode_map(cbor_value_t *dec, cbor_container_t *map);
uint32_t cbor_decode_map_size(cbor_value_t *dec, cbor_container_t *map);

cbor_result_t cbor_decode_uint8_t(cbor_value_t *dec, uint8_t *val);
cbor_result_t cbor_decode_uint16_t(cbor_value_t *dec, uint16_t *val);
cbor_result_t cbor_decode_uint32_t(cbor_value_t *dec, uint32_t *val);

cbor_result_t cbor_decode_int8_t(cbor_value_t *dec, int8_t *val);
cbor_result_t cbor_decode_int16_t(cbor_value_t *dec, int16_t *val);
cbor_result_t cbor_decode_int32_t(cbor_value_t *dec, int32_t *val);

cbor_result_t cbor_decode_float(cbor_value_t *dec, float *val);

cbor_result_t cbor_decode_bstr(cbor_value_t *dec, const uint8_t **buf, uint32_t *len);
cbor_result_t cbor_decode_tstr(cbor_value_t *dec, const uint8_t **buf, uint32_t *len);

cbor_result_t cbor_decode_tag(cbor_value_t *dec, uint32_t *val);
cbor_result_t cbor_decode_bool(cbor_value_t *dec, bool *val);

void cbor_encoder_init(cbor_value_t *enc, uint8_t *data, uint32_t len);
uint32_t cbor_encoder_len(cbor_value_t *enc);

cbor_result_t cbor_encode_array(cbor_value_t *enc, uint32_t len);
cbor_result_t cbor_encode_map(cbor_value_t *enc, uint32_t len);

cbor_result_t cbor_encode_array_indefinite(cbor_value_t *enc);
cbor_result_t cbor_encode_map_indefinite(cbor_value_t *enc);
cbor_result_t cbor_encode_end_indefinite(cbor_value_t *enc);

cbor_result_t cbor_encode_uint8_t(cbor_value_t *enc, const uint8_t *val);
cbor_result_t cbor_encode_uint16_t(cbor_value_t *enc, const uint16_t *val);
cbor_result_t cbor_encode_uint32_t(cbor_value_t *enc, const uint32_t *val);

cbor_result_t cbor_encode_int8_t(cbor_value_t *enc, const int8_t *val);
cbor_result_t cbor_encode_int16_t(cbor_value_t *enc, const int16_t *val);
cbor_result_t cbor_encode_int32_t(cbor_value_t *enc, const int32_t *val);

cbor_result_t cbor_encode_float(cbor_value_t *enc, const float *val);

cbor_result_t cbor_encode_bstr(cbor_value_t *enc, const uint8_t *buf, uint32_t len);
cbor_result_t cbor_encode_tstr(cbor_value_t *enc, const uint8_t *buf, uint32_t len);
cbor_result_t cbor_encode_str(cbor_value_t *enc, const char *buf);

cbor_result_t cbor_encode_tag(cbor_value_t *enc, const uint32_t *val);
cbor_result_t cbor_encode_bool(cbor_value_t *enc, const bool *val);