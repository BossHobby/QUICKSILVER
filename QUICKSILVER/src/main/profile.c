#include "profile.h"

#include <string.h>

#include "drv_usb.h"

profile_t profile = {
    .silverware_rate = {
        .max_rate = {
            MAX_RATE,
            MAX_RATE,
            MAX_RATEYAW,
        },
        .acro_expo = {
            ACRO_EXPO_ROLL,
            ACRO_EXPO_PITCH,
            ACRO_EXPO_YAW,
        },
        .angle_expo = {
            ANGLE_EXPO_ROLL,
            ANGLE_EXPO_PITCH,
            ANGLE_EXPO_YAW,
        },
    },
    .betaflight_rate = {
        .rc_rate = {
            BF_RC_RATE_ROLL,
            BF_RC_RATE_PITCH,
            BF_RC_RATE_YAW,
        },
        .super_rate = {
            BF_SUPER_RATE_ROLL,
            BF_SUPER_RATE_PITCH,
            BF_SUPER_RATE_YAW,
        },
        .expo = {
            BF_EXPO_ROLL,
            BF_EXPO_PITCH,
            BF_EXPO_YAW,
        },
    },

#ifdef SILVERWARE_RATES
    .rate_mode = RATE_MODE_SILVERWARE,
#endif
#ifdef BETAFLIGHT_RATES
    .rate_mode = RATE_MODE_BETAFLIGHT,
#endif
    .level_max_angle = LEVEL_MAX_ANGLE,
    .low_rate_mulitplier = LOW_RATES_MULTI,
    .sticks_deadband = STICKS_DEADBAND,
};

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2) {
  if (len2 != len1) {
    return 0;
  }
  for (size_t i = 0; i < len1; i++) {
    if (str1[i] != str2[i]) {
      return 0;
    }
  }
  return 1;
}

int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2) {
  return buf_equal(str1, len1, (const uint8_t *)str2, strlen(str2));
}

cbor_result_t cbor_encode_vector_t(cbor_value_t *enc, vector_t vec) {
  cbor_result_t res = cbor_encode_array(enc, 3);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, vec.axis[0]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, vec.axis[1]);
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_float(enc, vec.axis[2]);
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

#define START_STRUCT_ENCODER(type)                              \
  cbor_result_t cbor_encode_##type(cbor_value_t *enc, type o) { \
    cbor_result_t res = CBOR_OK;                                \
    res = cbor_encode_map_indefinite(enc);                      \
    if (res < CBOR_OK)                                          \
      return res;

#define END_STRUCT_ENCODER()              \
  return cbor_encode_end_indefinite(enc); \
  }

#define MEMBER(member, type)               \
  res = cbor_encode_str(enc, #member);     \
  if (res < CBOR_OK)                       \
    return res;                            \
  res = cbor_encode_##type(enc, o.member); \
  if (res < CBOR_OK)                       \
    return res;

START_STRUCT_ENCODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_ENCODER()

START_STRUCT_ENCODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_ENCODER()
#undef MEMBER

cbor_result_t cbor_decode_vector_t(cbor_value_t *it, vector_t *vec) {
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

#define START_STRUCT_DECODER(type)                                   \
  cbor_result_t cbor_decode_##type(cbor_value_t *dec, type *o) {     \
    cbor_result_t res = CBOR_OK;                                     \
    cbor_container_t map;                                            \
    res = cbor_decode_map(dec, &map);                                \
    if (res < CBOR_OK)                                               \
      return res;                                                    \
    const uint8_t *name;                                             \
    uint32_t name_len;                                               \
    for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) { \
      res = cbor_decode_tstr(dec, &name, &name_len);                 \
      if (res < CBOR_OK)                                             \
        return res;

#define END_STRUCT_DECODER()   \
  res = cbor_decode_skip(dec); \
  if (res < CBOR_OK)           \
    return res;                \
  }                            \
  return res;                  \
  }

#define MEMBER(member, type)                       \
  if (buf_equal_string(name, name_len, #member)) { \
    res = cbor_decode_##type(dec, &o->member);     \
    if (res < CBOR_OK)                             \
      return res;                                  \
    continue;                                      \
  }
START_STRUCT_DECODER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_DECODER()

START_STRUCT_DECODER(profile_t)
PROFILE_MEMBERS
END_STRUCT_DECODER()
#undef MEMBER