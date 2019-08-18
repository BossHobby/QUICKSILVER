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

    .low_rate_mulitplier = LOW_RATES_MULTI,
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

int nanocbor_fmt_uint8(nanocbor_encoder_t *enc, uint8_t v) {
  return nanocbor_fmt_uint(enc, v);
}

void nanocbor_fmt_vector_t(nanocbor_encoder_t *enc, vector_t vec) {
  nanocbor_fmt_array(enc, 3);
  nanocbor_fmt_float(enc, vec.axis[0]);
  nanocbor_fmt_float(enc, vec.axis[1]);
  nanocbor_fmt_float(enc, vec.axis[2]);
}

#define START_STRUCT_GETTER(type)                             \
  void nanocbor_fmt_##type(nanocbor_encoder_t *enc, type o) { \
    nanocbor_fmt_map_indefinite(enc);

#define END_STRUCT_GETTER()         \
  nanocbor_fmt_end_indefinite(enc); \
  }

#define MEMBER(member, type)       \
  nanocbor_put_tstr(enc, #member); \
  nanocbor_fmt_##type(enc, o.member);

START_STRUCT_GETTER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_GETTER()

START_STRUCT_GETTER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_GETTER()

START_STRUCT_GETTER(profile_t)
PROFILE_MEMBERS
END_STRUCT_GETTER()
#undef MEMBER

void nanocbor_get_vector_t(nanocbor_value_t *it, vector_t *vec) {
  nanocbor_value_t array;
  nanocbor_enter_array(it, &array);
  nanocbor_get_float(&array, &vec->axis[0]);
  nanocbor_get_float(&array, &vec->axis[1]);
  nanocbor_get_float(&array, &vec->axis[2]);
  nanocbor_leave_container(it, &array);
}

#define START_STRUCT_SETTER(type)                                    \
  void nanocbor_get_##type(nanocbor_value_t *it, type *o) {          \
    nanocbor_value_t map;                                            \
    if (nanocbor_enter_map(it, &map) < NANOCBOR_OK) {                \
      usb_serial_print("CBOR ERROR\r\n");                            \
    }                                                                \
    const uint8_t *name;                                             \
    size_t name_len;                                                 \
    while (!nanocbor_at_end(&map)) {                                 \
      if (nanocbor_get_tstr(&map, &name, &name_len) < NANOCBOR_OK) { \
        usb_serial_print("CBOR ERROR\r\n");                          \
      }

#define END_STRUCT_SETTER()           \
  nanocbor_skip(&map);                \
  }                                   \
  nanocbor_leave_container(it, &map); \
  }

#define MEMBER(member, type)                       \
  if (buf_equal_string(name, name_len, #member)) { \
    nanocbor_get_##type(&map, &o->member);         \
    continue;                                      \
  }
START_STRUCT_SETTER(rate_mode_silverware_t)
SILVERWARE_RATE_MEMBERS
END_STRUCT_SETTER()

START_STRUCT_SETTER(rate_mode_betaflight_t)
BETAFLIGHT_RATE_MEMBERS
END_STRUCT_SETTER()

START_STRUCT_SETTER(profile_t)
PROFILE_MEMBERS
END_STRUCT_SETTER()
#undef MEMBER