#include "util/cbor_helper.h"

cbor_result_t cbor_encode_float_array(cbor_value_t *enc, const float *array, uint32_t size) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array(enc, size))

  for (uint32_t i = 0; i < size; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_encode_uint8_array(cbor_value_t *enc, const uint8_t *array, uint32_t size) {
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_encode_array(enc, size));

  for (uint32_t i = 0; i < size; i++) {
    CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_decode_float_array(cbor_value_t *enc, float *array, uint32_t size) {
  cbor_container_t container;
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_decode_array(enc, &container))

  for (uint32_t i = 0; i < size; i++) {
    CBOR_CHECK_ERROR(res = cbor_decode_float(enc, &array[i]));
  }

  return res;
}

cbor_result_t cbor_decode_uint8_array(cbor_value_t *enc, uint8_t *array, uint32_t size) {
  cbor_container_t container;
  CBOR_CHECK_ERROR(cbor_result_t res = cbor_decode_array(enc, &container));

  for (uint32_t i = 0; i < size; i++) {
    CBOR_CHECK_ERROR(res = cbor_decode_uint8(enc, &array[i]));
  }

  return res;
}