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

float vec3_magnitude(vec3_t *v) {
  float max = 0;
  for (uint8_t axis = 0; axis < 3; axis++) {
    max += v->axis[axis] * v->axis[axis];
  }
  return 1.0f / Q_rsqrt(max);
}

cbor_result_t cbor_encode_compact_vec3_t(cbor_value_t *enc, const compact_vec3_t *vec) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, 3));

  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[2]));

  return res;
}

cbor_result_t cbor_decode_compact_vec3_t(cbor_value_t *it, compact_vec3_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  CBOR_CHECK_ERROR(res = cbor_decode_array(it, &array));

  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[2]));

  return res;
}

void vec3_from_array(vec3_t *out, float *in) {
  out->axis[0] = in[0];
  out->axis[1] = in[1];
  out->axis[2] = in[2];
}

void vec3_compress(compact_vec3_t *out, vec3_t *in, float scale) {
  out->axis[0] = in->axis[0] * scale;
  out->axis[1] = in->axis[1] * scale;
  out->axis[2] = in->axis[2] * scale;
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

cbor_result_t cbor_encode_compact_vec4_t(cbor_value_t *enc, const compact_vec4_t *vec) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_array(enc, 4));

  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[2]));
  CBOR_CHECK_ERROR(res = cbor_encode_int16(enc, &vec->axis[3]));

  return res;
}

cbor_result_t cbor_decode_compact_vec4_t(cbor_value_t *it, compact_vec4_t *vec) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t array;
  CBOR_CHECK_ERROR(res = cbor_decode_array(it, &array));

  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[0]));
  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[1]));
  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[2]));
  CBOR_CHECK_ERROR(res = cbor_decode_int16(it, &vec->axis[3]));

  return res;
}

void vec4_from_array(vec4_t *out, float *in) {
  out->axis[0] = in[0];
  out->axis[1] = in[1];
  out->axis[2] = in[2];
  out->axis[3] = in[3];
}

void vec4_compress(compact_vec4_t *out, vec4_t *in, float scale) {
  out->axis[0] = in->axis[0] * scale;
  out->axis[1] = in->axis[1] * scale;
  out->axis[2] = in->axis[2] * scale;
  out->axis[3] = in->axis[3] * scale;
}