#pragma once

#include <stdbool.h>

#include <cbor.h>

#include "util/util.h"

cbor_result_t cbor_handle_error(cbor_result_t err);

#define CBOR_CHECK_ERROR(expr)     \
  expr;                            \
  if (res < CBOR_OK) {             \
    return cbor_handle_error(res); \
  }

#define CBOR_START_STRUCT_ENCODER(type)                                \
  cbor_result_t cbor_encode_##type(cbor_value_t *enc, const type *o) { \
    cbor_result_t res = CBOR_OK;                                       \
    CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

#define CBOR_END_STRUCT_ENCODER()         \
  return cbor_encode_end_indefinite(enc); \
  }

#define CBOR_ENCODE_MEMBER(member, type)                 \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member)); \
  CBOR_CHECK_ERROR(res = cbor_encode_##type(enc, &o->member));

#define CBOR_ENCODE_STR_MEMBER(member)                   \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member)); \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, o->member));

#define CBOR_ENCODE_TSTR_MEMBER(member, size)            \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member)); \
  CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, o->member, size));

#define CBOR_ENCODE_BSTR_MEMBER(member, size)            \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member)); \
  CBOR_CHECK_ERROR(res = cbor_encode_bstr(enc, o->member, size));

#define CBOR_ENCODE_ARRAY_MEMBER(member, size, type)                \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member));            \
  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, size));             \
  for (uint32_t i = 0; i < size; i++) {                             \
    CBOR_CHECK_ERROR(res = cbor_encode_##type(enc, &o->member[i])); \
  }

#define CBOR_ENCODE_INDEX_ARRAY_MEMBER(member, size, type)          \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member));            \
  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, size - 1));         \
  for (uint32_t i = 1; i < size; i++) {                             \
    CBOR_CHECK_ERROR(res = cbor_encode_##type(enc, &o->member[i])); \
  }

#define CBOR_ENCODE_STR_ARRAY_MEMBER(member, size)              \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member));        \
  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, size));         \
  for (uint32_t i = 0; i < size; i++) {                         \
    CBOR_CHECK_ERROR(res = cbor_encode_str(enc, o->member[i])); \
  }

#define CBOR_ENCODE_TSTR_ARRAY_MEMBER(member, size, str_size)                               \
  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, #member));                                    \
  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, size));                                     \
  for (uint32_t i = 0; i < size; i++) {                                                     \
    CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, (const uint8_t *)o->member[i], str_size)); \
  }

#define CBOR_START_STRUCT_DECODER(type)                              \
  cbor_result_t cbor_decode_##type(cbor_value_t *dec, type *o) {     \
    cbor_result_t res = CBOR_OK;                                     \
    cbor_container_t map;                                            \
    CBOR_CHECK_ERROR(res = cbor_decode_map(dec, &map));              \
    const uint8_t *name;                                             \
    uint32_t name_len;                                               \
    for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) { \
      CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));

#define CBOR_END_STRUCT_DECODER()                \
  CBOR_CHECK_ERROR(res = cbor_decode_skip(dec)); \
  }                                              \
  return res;                                    \
  }

#define CBOR_DECODE_MEMBER(member, type)                         \
  if (buf_equal_string(name, name_len, #member)) {               \
    CBOR_CHECK_ERROR(res = cbor_decode_##type(dec, &o->member)); \
    continue;                                                    \
  }

#define CBOR_DECODE_STR_MEMBER(member)                       \
  if (buf_equal_string(name, name_len, #member)) {           \
    CBOR_CHECK_ERROR(res = cbor_decode_str(dec, o->member)); \
    continue;                                                \
  }

#define CBOR_DECODE_TSTR_MEMBER(member, size)                            \
  if (buf_equal_string(name, name_len, #member)) {                       \
    CBOR_CHECK_ERROR(res = cbor_decode_tstr_copy(dec, o->member, size)); \
    continue;                                                            \
  }

#define CBOR_DECODE_BSTR_MEMBER(member, size)                            \
  if (buf_equal_string(name, name_len, #member)) {                       \
    CBOR_CHECK_ERROR(res = cbor_decode_bstr_copy(dec, o->member, size)); \
    continue;                                                            \
  }

#define CBOR_DECODE_ARRAY_MEMBER(member, size, type)                                \
  if (buf_equal_string(name, name_len, #member)) {                                  \
    cbor_container_t array;                                                         \
    CBOR_CHECK_ERROR(res = cbor_decode_array(dec, &array));                         \
    for (uint32_t i = 0; i < min(size, cbor_decode_array_size(dec, &array)); i++) { \
      CBOR_CHECK_ERROR(res = cbor_decode_##type(dec, &o->member[i]));               \
    }                                                                               \
    continue;                                                                       \
  }

#define CBOR_DECODE_INDEX_ARRAY_MEMBER(member, size, type)                          \
  if (buf_equal_string(name, name_len, #member)) {                                  \
    cbor_container_t array;                                                         \
    CBOR_CHECK_ERROR(res = cbor_decode_array(dec, &array));                         \
    for (uint32_t i = 0; i < min(size, cbor_decode_array_size(dec, &array)); i++) { \
      type tmp = {};                                                                \
      CBOR_CHECK_ERROR(res = cbor_decode_##type(dec, &tmp));                        \
      o->member[tmp.index] = tmp;                                                   \
    }                                                                               \
    continue;                                                                       \
  }

#define CBOR_DECODE_STR_ARRAY_MEMBER(member, size)                                  \
  if (buf_equal_string(name, name_len, #member)) {                                  \
    cbor_container_t array;                                                         \
    CBOR_CHECK_ERROR(res = cbor_decode_array(dec, &array));                         \
    for (uint32_t i = 0; i < min(size, cbor_decode_array_size(dec, &array)); i++) { \
      CBOR_CHECK_ERROR(res = cbor_decode_str(dec, &o->member[i]));                  \
    }                                                                               \
    continue;                                                                       \
  }

#define CBOR_DECODE_TSTR_ARRAY_MEMBER(member, size, str_size)                                \
  if (buf_equal_string(name, name_len, #member)) {                                           \
    cbor_container_t array;                                                                  \
    CBOR_CHECK_ERROR(res = cbor_decode_array(dec, &array));                                  \
    for (uint32_t i = 0; i < min(size, cbor_decode_array_size(dec, &array)); i++) {          \
      CBOR_CHECK_ERROR(res = cbor_decode_tstr_copy(dec, (uint8_t *)o->member[i], str_size)); \
    }                                                                                        \
    continue;                                                                                \
  }

cbor_result_t cbor_encode_float_array(cbor_value_t *enc, const float *array, uint32_t size);
cbor_result_t cbor_encode_uint8_array(cbor_value_t *enc, const uint8_t *array, uint32_t size);

cbor_result_t cbor_decode_float_array(cbor_value_t *enc, float *array, uint32_t size);
cbor_result_t cbor_decode_uint8_array(cbor_value_t *enc, uint8_t *array, uint32_t size);

cbor_result_t cbor_decode_tstr_copy(cbor_value_t *dec, uint8_t *buf, uint32_t size);
cbor_result_t cbor_decode_bstr_copy(cbor_value_t *dec, uint8_t *buf, uint32_t size);
