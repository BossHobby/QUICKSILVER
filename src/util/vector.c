#include "vector.h"

cbor_result_t cbor_encode_vec3_t(cbor_value_t *enc, const vec3_t *vec) {
  cbor_result_t res = cbor_encode_array(enc, 3);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[0]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[1]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, &vec->axis[2]);
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

cbor_result_t cbor_decode_vec3_t(cbor_value_t *it, vec3_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  res = cbor_decode_array(it, &array);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[0]);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[1]);
  if (res < CBOR_OK)
    return res;

  res = cbor_decode_float(it, &vec->axis[2]);
  if (res < CBOR_OK)
    return res;

  return res;
}