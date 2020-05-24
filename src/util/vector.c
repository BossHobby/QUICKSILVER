#include "vector.h"

#include "cbor_helper.h"

cbor_result_t cbor_encode_vec3_t(cbor_value_t *enc, const vec3_t *vec) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[2]));

  return res;
}

cbor_result_t cbor_decode_vec3_t(cbor_value_t *it, vec3_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  CBOR_CHECK_ERROR(res = cbor_decode_array(it, &array));

  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[2]));

  return res;
}

cbor_result_t cbor_encode_vec4_t(cbor_value_t *enc, const vec4_t *vec) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, 4));

  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[2]));
  CBOR_CHECK_ERROR(res = cbor_encode_float(enc, &vec->axis[3]));

  return res;
}

cbor_result_t cbor_decode_vec4_t(cbor_value_t *it, vec4_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  CBOR_CHECK_ERROR(res = cbor_decode_array(it, &array));

  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[2]));
  CBOR_CHECK_ERROR(res = cbor_decode_float(it, &vec->axis[3]));

  return res;
}