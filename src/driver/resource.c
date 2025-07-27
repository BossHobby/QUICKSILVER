#include "resource.h"

#include <stdio.h>
#include <stdlib.h>

#include "util/cbor_helper.h"
#include "util/util.h"

#include "timer.h"

#define check_str(str)                                                   \
  ({                                                                     \
    bool match = buf_equal_string((uint8_t *)end, sizeof(str) - 1, str); \
    if (match)                                                           \
      end += sizeof(str) - 1;                                            \
    match;                                                               \
  })

cbor_result_t cbor_decode_resource_tag_t(cbor_value_t *dec, resource_tag_t *t) {
  const uint8_t *name;
  uint32_t name_len;

  CBOR_CHECK_ERROR(cbor_result_t res = cbor_decode_tstr(dec, &name, &name_len));

  char *end = (char *)name;
  if (check_str("SPI")) {
    const int32_t index = strtol(end, &end, 10);
    if (check_str("_MOSI")) {
      *t = SPI_TAG(index, RES_SPI_MOSI);
    } else if (check_str("_MISO")) {
      *t = SPI_TAG(index, RES_SPI_MISO);
    } else if (check_str("_SCK")) {
      *t = SPI_TAG(index, RES_SPI_SCK);
    } else {
      return CBOR_ERR_INVALID_TYPE;
    }
  } else if (check_str("TIMER")) {
    int32_t index = TIMER_INVALID;
    const int32_t num = strtol(end, &end, 10);
    switch (num) {
#define TIMER(_num)      \
  case _num:             \
    index = TIMER##_num; \
    break;
      TIMERS
#undef TIMER
    }
    if (!check_str("_CH")) {
      return CBOR_ERR_INVALID_TYPE;
    }
    switch (*end) {
    case '1':
      *t = TIMER_TAG(index, TIMER_CH1);
      break;
    case '2':
      *t = TIMER_TAG(index, TIMER_CH2);
      break;
    case '3':
      *t = TIMER_TAG(index, TIMER_CH3);
      break;
    case '4':
      *t = TIMER_TAG(index, TIMER_CH4);
      break;
    default:
      return CBOR_ERR_INVALID_TYPE;
    };
  } else {
    *t = RESOURCE_INVALID;
  }

  return res;
}

cbor_result_t cbor_encode_resource_tag_t(cbor_value_t *enc, const resource_tag_t *d) {
  char tag[64];
  int len = 0;

  switch (RESOURCE_TAG_TYPE(*d)) {
  case RESOURCE_SPI:
    switch (SPI_TAG_PIN(*d)) {
    case RES_SPI_MOSI:
      len = snprintf(tag, 64, "SPI%d_MOSI", SPI_TAG_PORT(*d));
      break;
    case RES_SPI_MISO:
      len = snprintf(tag, 64, "SPI%d_MISO", SPI_TAG_PORT(*d));
      break;
    case RES_SPI_SCK:
      len = snprintf(tag, 64, "SPI%d_SCK", SPI_TAG_PORT(*d));
      break;
    default:
      break;
    }
    break;
  case RESOURCE_TIM:
    switch (TIMER_TAG_CH(*d)) {
    case TIMER_CH1:
    case TIMER_CH1N:
      len = snprintf(tag, 64, "TIMER%d_CH1", TIMER_TAG_TIM(*d));
      break;
    case TIMER_CH2:
    case TIMER_CH2N:
      len = snprintf(tag, 64, "TIMER%d_CH2", TIMER_TAG_TIM(*d));
      break;
    case TIMER_CH3:
    case TIMER_CH3N:
      len = snprintf(tag, 64, "TIMER%d_CH3", TIMER_TAG_TIM(*d));
      break;
    case TIMER_CH4:
    case TIMER_CH4N:
      len = snprintf(tag, 64, "TIMER%d_CH4", TIMER_TAG_TIM(*d));
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }

  return cbor_encode_tstr(enc, (uint8_t *)tag, len);
}