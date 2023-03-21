#pragma once

#include <cbor.h>

typedef struct {
  uint8_t name[32];
} target_t;

#define TARGET_MEMBERS \
  TSTR_MEMBER(name, 32)

extern target_t target;

cbor_result_t cbor_encode_target_t(cbor_value_t *enc, const target_t *t);
cbor_result_t cbor_decode_target_t(cbor_value_t *dec, target_t *t);