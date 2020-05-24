#pragma once

#include <cbor.h>

typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
  };
  float axis[3];
} vec3_t;

cbor_result_t cbor_encode_vec3_t(cbor_value_t *enc, const vec3_t *vec);
cbor_result_t cbor_decode_vec3_t(cbor_value_t *dec, vec3_t *vec);

typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
    float throttle;
  };
  float axis[4];
} vec4_t;

cbor_result_t cbor_encode_vec4_t(cbor_value_t *enc, const vec4_t *vec);
cbor_result_t cbor_decode_vec4_t(cbor_value_t *dec, vec4_t *vec);