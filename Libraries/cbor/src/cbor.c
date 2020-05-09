#include "cbor.h"

#define __USE_MISC

#include <math.h>
#include <string.h>

#define CBOR_TYPE_OFFSET (5)
#define CBOR_TYPE_MASK 0xE0
#define CBOR_VALUE_MASK 0x1F

void cbor_decoder_init(cbor_value_t *dec, uint8_t *data, uint32_t len) {
  dec->start = dec->curr = data;
  dec->end = dec->start + len;
}

void cbor_encoder_init(cbor_value_t *enc, uint8_t *data, uint32_t len) {
  memset(data, 0, len);

  enc->start = enc->curr = data;
  enc->end = enc->start + len;
}

static int32_t _cbor_remaining(cbor_value_t *dec) {
  return (int32_t)(dec->end - dec->curr);
}

uint32_t cbor_encoder_len(cbor_value_t *enc) {
  return (uint32_t)(enc->curr - enc->start);
}

static cbor_result_t _cbor_decode_type(uint8_t v) {
  return (v & CBOR_TYPE_MASK) >> CBOR_TYPE_OFFSET;
}

static cbor_result_t _cbor_decode_ensure_type(cbor_value_t *dec, cbor_major_type_t type) {
  if (_cbor_remaining(dec) <= 0) {
    return CBOR_ERR_EOF;
  }
  cbor_result_t value = _cbor_decode_type(*dec->curr);
  if ((cbor_major_type_t)value != type) {
    return CBOR_ERR_INVALID_TYPE;
  }
  return value;
}

cbor_result_t cbor_decode_type(cbor_value_t *dec) {
  if (_cbor_remaining(dec) <= 0) {
    return CBOR_ERR_EOF;
  }
  return _cbor_decode_type(*dec->curr);
}

static cbor_result_t _cbor_decode_flag(uint8_t v) {
  return (v & CBOR_VALUE_MASK);
}

cbor_result_t cbor_decode_flag(cbor_value_t *dec) {
  if (_cbor_remaining(dec) <= 0) {
    return CBOR_ERR_EOF;
  }
  return _cbor_decode_flag(*dec->curr);
}

static cbor_result_t _cbor_advance(cbor_value_t *dec, cbor_result_t res) {
  if (res > 0) {
    dec->curr += res;
  }
  return res;
}

static cbor_size_type_t _cbor_size_for_value(uint32_t val) {
  if (val <= UINT8_MAX) {
    return CBOR_SIZE_BYTE;
  } else if (val <= UINT16_MAX) {
    return CBOR_SIZE_SHORT;
  } else if (val <= UINT32_MAX) {
    return CBOR_SIZE_WORD;
  }
  return CBOR_SIZE_LONG;
}

static cbor_result_t _cbor_decode_raw(cbor_value_t *dec, uint8_t *val, uint8_t max) {
  uint8_t byte_len = *dec->curr & CBOR_VALUE_MASK;
  if (byte_len > max) {
    return CBOR_ERR_OVERFLOW;
  }

  uint8_t size = (uint8_t)(1 << (max - CBOR_SIZE_BYTE));
  memset(val, 0, size);
  if (byte_len < CBOR_SIZE_BYTE) {
    *val = byte_len;
    return 1;
  }

  uint8_t bytes = (uint8_t)(1 << (byte_len - CBOR_SIZE_BYTE));
  if ((dec->curr + bytes) >= dec->end) {
    return CBOR_ERR_EOF;
  }

  memcpy(val + size - bytes, dec->curr + 1, bytes);
  for (uint8_t i = 0, j = (uint8_t)(size - 1U); i < j; i++, j--) {
    uint8_t t = val[j];
    val[j] = val[i];
    val[i] = t;
  }

  return (cbor_result_t)(1 + bytes);
}

static cbor_result_t _cbor_encode_raw(cbor_value_t *enc, cbor_major_type_t type, const uint8_t *val, uint8_t max) {
  uint8_t byte_len = max;
  if ((enc->curr + 1) >= enc->end) {
    return CBOR_ERR_EOF;
  }
  if (max == CBOR_SIZE_BYTE && *val < CBOR_SIZE_BYTE) {
    *enc->curr++ = (uint8_t)((type << CBOR_TYPE_OFFSET) | (*val & CBOR_VALUE_MASK));
    return 1;
  }

  *enc->curr++ = (uint8_t)((type << CBOR_TYPE_OFFSET) | (byte_len & CBOR_VALUE_MASK));

  uint8_t size = (uint8_t)(1 << (max - CBOR_SIZE_BYTE));
  if ((enc->curr + size) >= enc->end) {
    return CBOR_ERR_EOF;
  }
  memcpy(enc->curr, val, size);
  for (uint8_t i = 0, j = (uint8_t)(size - 1U); i < j; i++, j--) {
    uint8_t t = enc->curr[j];
    enc->curr[j] = enc->curr[i];
    enc->curr[i] = t;
  }
  enc->curr += size;

  return (cbor_result_t)(1 + size);
}

cbor_result_t cbor_decode_skip(cbor_value_t *dec) {
  cbor_result_t type = cbor_decode_type(dec);
  if (type < 0) {
    return type;
  }

  const uint8_t *tmp;
  uint32_t dummy = 0;

  switch (type) {
  case CBOR_TYPE_BSTR:
    return cbor_decode_bstr(dec, &tmp, &dummy);
  case CBOR_TYPE_TSTR:
    return cbor_decode_tstr(dec, &tmp, &dummy);
  case CBOR_TYPE_ARRAY: {
    cbor_container_t array;
    cbor_result_t res = cbor_decode_array(dec, &array);
    if (res < CBOR_OK) {
      return res;
    }
    for (uint64_t i = 0; i < cbor_decode_array_size(dec, &array); i++) {
      cbor_result_t skip_res = cbor_decode_skip(dec);
      if (skip_res < CBOR_OK) {
        return skip_res;
      }
    }
    return CBOR_OK;
  }
  case CBOR_TYPE_MAP: {
    cbor_container_t map;
    cbor_result_t res = cbor_decode_map(dec, &map);
    if (res < CBOR_OK) {
      return res;
    }
    for (uint64_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
      cbor_result_t key_res = cbor_decode_skip(dec);
      if (key_res < CBOR_OK) {
        return key_res;
      }
      cbor_result_t value_res = cbor_decode_skip(dec);
      if (value_res < CBOR_OK) {
        return value_res;
      }
    }
    return CBOR_OK;
  }
  default:
    return _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)&dummy, CBOR_SIZE_WORD));
  }
}

cbor_result_t cbor_decode_array(cbor_value_t *dec, cbor_container_t *array) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_ARRAY);
  if (type < 0) {
    return type;
  }

  array->dec = dec;
  array->size = 0;
  array->is_streaming = 0;

  cbor_result_t flag = cbor_decode_flag(dec);
  if (flag == CBOR_SIZE_INDEFINITE) {
    array->is_streaming = 1;
    return _cbor_advance(dec, 1);
  }

  cbor_result_t res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)&array->size, CBOR_SIZE_WORD));
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

uint32_t cbor_decode_array_size(cbor_value_t *dec, cbor_container_t *array) {
  if (array->is_streaming == 0) {
    // simple array
    return array->size;
  }

  // OK, we have a CBOR_SIZE_INDEFINITE array
  // Check for a <break> float first
  cbor_result_t type = cbor_decode_type(dec);
  if (type < 0) {
    return (uint32_t)type;
  }
  cbor_result_t flag = cbor_decode_flag(dec);
  if (type == CBOR_TYPE_FLOAT && flag == CBOR_SIZE_INDEFINITE) {
    // break flag, we are done here
    _cbor_advance(dec, 1);
    array->is_streaming = 0;
    return 0;
  }

  // CBOR_SIZE_INDEFINITE array, no break,
  return UINT32_MAX;
}

cbor_result_t cbor_decode_map(cbor_value_t *dec, cbor_container_t *map) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_MAP);
  if (type < 0) {
    return type;
  }

  map->dec = dec;
  map->size = 0;
  map->is_streaming = 0;

  cbor_result_t flag = cbor_decode_flag(dec);
  if (flag == CBOR_SIZE_INDEFINITE) {
    map->is_streaming = 1;
    return _cbor_advance(dec, 1);
  }

  cbor_result_t res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)&map->size, CBOR_SIZE_WORD));
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

uint32_t cbor_decode_map_size(cbor_value_t *dec, cbor_container_t *map) {
  if (map->is_streaming == 0) {
    // simple, empty map
    return map->size;
  }

  // OK, we have a CBOR_SIZE_INDEFINITE map
  // Check for a <break> float first
  cbor_result_t type = cbor_decode_type(dec);
  if (type < 0) {
    return (uint32_t)type;
  }
  cbor_result_t flag = cbor_decode_flag(dec);
  if (type == CBOR_TYPE_FLOAT && flag == CBOR_SIZE_INDEFINITE) {
    // break flag, we are done here
    _cbor_advance(dec, 1);
    map->is_streaming = 0;
    return 0;
  }

  // CBOR_SIZE_INDEFINITE map, no break,
  return UINT32_MAX;
}

cbor_result_t cbor_decode_uint8(cbor_value_t *dec, uint8_t *val) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_UINT);
  if (type < 0) {
    return type;
  }
  return _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_BYTE));
}
cbor_result_t cbor_decode_uint16(cbor_value_t *dec, uint16_t *val) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_UINT);
  if (type < 0) {
    return type;
  }
  return _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_SHORT));
}
cbor_result_t cbor_decode_uint32(cbor_value_t *dec, uint32_t *val) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_UINT);
  if (type < 0) {
    return type;
  }
  return _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_WORD));
}

cbor_result_t cbor_decode_int8(cbor_value_t *dec, int8_t *val) {
  cbor_result_t res = _cbor_decode_ensure_type(dec, CBOR_TYPE_NINT);
  if (res < 0) {
    return res;
  }

  res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_BYTE));
  if (res < 0) {
    return res;
  }
  *val = (int8_t)(-1 - *((uint8_t *)(val)));
  return res;
}
cbor_result_t cbor_decode_int16(cbor_value_t *dec, int16_t *val) {
  cbor_result_t res = _cbor_decode_ensure_type(dec, CBOR_TYPE_NINT);
  if (res < 0) {
    return res;
  }

  res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_SHORT));
  if (res < 0) {
    return res;
  }
  *val = (int16_t)(-1 - *((uint16_t *)(val)));
  return res;
}
cbor_result_t cbor_decode_int32(cbor_value_t *dec, int32_t *val) {
  cbor_result_t res = _cbor_decode_ensure_type(dec, CBOR_TYPE_NINT);
  if (res < 0) {
    return res;
  }

  res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_WORD));
  if (res < 0) {
    return res;
  }
  *val = (int32_t)(-1 - *((uint32_t *)(val)));
  return res;
}

float _cbor_decode_half_float(uint32_t half) {
  int32_t exp = (half >> 10) & 0x1f;
  int32_t mant = half & 0x3ff;

  float val;
  if (exp == 0)
    val = ldexpf((float)mant, -24);
  else if (exp != 31)
    val = ldexpf((float)(mant + 1024), exp - 25);
  else
    val = mant == 0 ? INFINITY : NAN;
  return half & 0x8000 ? -val : val;
}

cbor_result_t cbor_decode_float(cbor_value_t *dec, float *val) {
  cbor_result_t res = _cbor_decode_ensure_type(dec, CBOR_TYPE_FLOAT);
  if (res < CBOR_OK) {
    return res;
  }

  uint32_t tmp = 0;
  cbor_result_t size = _cbor_decode_raw(dec, (uint8_t *)&tmp, CBOR_SIZE_WORD);
  if (size < CBOR_OK) {
    return size;
  }

  uint8_t byte_len = (uint8_t)cbor_decode_flag(dec);
  if (byte_len == CBOR_SIZE_SHORT) {
    *val = _cbor_decode_half_float(tmp);
  } else if (byte_len == CBOR_SIZE_WORD) {
    *((uint32_t *)val) = tmp;
  }

  return _cbor_advance(dec, size);
}

cbor_result_t cbor_decode_bstr(cbor_value_t *dec, const uint8_t **buf, uint32_t *len) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_BSTR);
  if (type < 0) {
    return type;
  }

  cbor_result_t res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)len, CBOR_SIZE_WORD));
  if (res < CBOR_OK) {
    return res;
  }
  if (_cbor_remaining(dec) < (int32_t)*len) {
    return CBOR_ERR_EOF;
  }
  if (res > 0) {
    *buf = (dec->curr);
    _cbor_advance(dec, (int32_t)*len);
  }
  return res;
}
cbor_result_t cbor_decode_tstr(cbor_value_t *dec, const uint8_t **buf, uint32_t *len) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_TSTR);
  if (type < 0) {
    return type;
  }

  cbor_result_t res = _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)len, CBOR_SIZE_WORD));
  if (res < CBOR_OK) {
    return res;
  }
  if (_cbor_remaining(dec) < (int32_t)*len) {
    return CBOR_ERR_EOF;
  }
  if (res > 0) {
    *buf = (dec->curr);
    _cbor_advance(dec, (int32_t)*len);
  }
  return res;
}

cbor_result_t cbor_decode_tag(cbor_value_t *dec, uint32_t *val) {
  cbor_result_t type = _cbor_decode_ensure_type(dec, CBOR_TYPE_TAG);
  if (type < 0) {
    return type;
  }
  return _cbor_advance(dec, _cbor_decode_raw(dec, (uint8_t *)val, CBOR_SIZE_WORD));
}

cbor_result_t cbor_encode_array(cbor_value_t *enc, uint32_t len) {
  cbor_result_t res = _cbor_encode_raw(enc, CBOR_TYPE_ARRAY, (uint8_t *)&len, _cbor_size_for_value(len));
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}
cbor_result_t cbor_encode_map(cbor_value_t *enc, uint32_t len) {
  cbor_result_t res = _cbor_encode_raw(enc, CBOR_TYPE_MAP, (uint8_t *)&len, _cbor_size_for_value(len));
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}
cbor_result_t cbor_encode_array_indefinite(cbor_value_t *enc) {
  if ((enc->curr + 1) >= enc->end) {
    return CBOR_ERR_EOF;
  }
  *enc->curr++ = (uint8_t)((CBOR_TYPE_ARRAY << CBOR_TYPE_OFFSET) | (CBOR_SIZE_INDEFINITE & CBOR_VALUE_MASK));
  return CBOR_OK;
}
cbor_result_t cbor_encode_map_indefinite(cbor_value_t *enc) {
  if ((enc->curr + 1) >= enc->end) {
    return CBOR_ERR_EOF;
  }
  *enc->curr++ = (uint8_t)((CBOR_TYPE_MAP << CBOR_TYPE_OFFSET) | (CBOR_SIZE_INDEFINITE & CBOR_VALUE_MASK));
  return CBOR_OK;
}
cbor_result_t cbor_encode_end_indefinite(cbor_value_t *enc) {
  if ((enc->curr + 1) >= enc->end) {
    return CBOR_ERR_EOF;
  }
  *enc->curr++ = (uint8_t)((CBOR_TYPE_FLOAT << CBOR_TYPE_OFFSET) | (CBOR_SIZE_INDEFINITE & CBOR_VALUE_MASK));
  return CBOR_OK;
}

cbor_result_t cbor_encode_uint8(cbor_value_t *enc, const uint8_t *val) {
  return _cbor_encode_raw(enc, CBOR_TYPE_UINT, (const uint8_t *)val, _cbor_size_for_value(*val));
}
cbor_result_t cbor_encode_uint16(cbor_value_t *enc, const uint16_t *val) {
  return _cbor_encode_raw(enc, CBOR_TYPE_UINT, (const uint8_t *)val, _cbor_size_for_value(*val));
}
cbor_result_t cbor_encode_uint32(cbor_value_t *enc, const uint32_t *val) {
  return _cbor_encode_raw(enc, CBOR_TYPE_UINT, (const uint8_t *)val, _cbor_size_for_value(*val));
}

cbor_result_t cbor_encode_int8(cbor_value_t *enc, const int8_t *val) {
  uint8_t proxy = 0;
  proxy = (uint8_t)(-1 - *val);
  return _cbor_encode_raw(enc, CBOR_TYPE_NINT, (const uint8_t *)&proxy, _cbor_size_for_value(proxy));
}
cbor_result_t cbor_encode_int16(cbor_value_t *enc, const int16_t *val) {
  uint16_t proxy = 0;
  proxy = (uint16_t)(-1 - *val);
  return _cbor_encode_raw(enc, CBOR_TYPE_NINT, (const uint8_t *)&proxy, _cbor_size_for_value(proxy));
}
cbor_result_t cbor_encode_int32(cbor_value_t *enc, const int32_t *val) {
  uint32_t proxy = 0;
  proxy = (uint32_t)(-1 - *val);
  return _cbor_encode_raw(enc, CBOR_TYPE_NINT, (const uint8_t *)&proxy, _cbor_size_for_value(proxy));
}

cbor_result_t cbor_encode_float(cbor_value_t *dec, const float *val) {
  return _cbor_encode_raw(dec, CBOR_TYPE_FLOAT, (const uint8_t *)val, CBOR_SIZE_WORD);
}

cbor_result_t cbor_encode_bstr(cbor_value_t *enc, const uint8_t *buf, uint32_t len) {
  cbor_result_t res = _cbor_encode_raw(enc, CBOR_TYPE_BSTR, (uint8_t *)&len, _cbor_size_for_value(len));
  if (res < CBOR_OK) {
    return res;
  }
  if ((uint32_t)_cbor_remaining(enc) < len) {
    return CBOR_ERR_EOF;
  }
  if (res > 0) {
    memcpy(enc->curr, buf, len);
    enc->curr += len;
  }
  return res;
}
cbor_result_t cbor_encode_tstr(cbor_value_t *enc, const uint8_t *buf, uint32_t len) {
  cbor_result_t res = _cbor_encode_raw(enc, CBOR_TYPE_TSTR, (uint8_t *)&len, _cbor_size_for_value(len));
  if (res < CBOR_OK) {
    return res;
  }
  if ((uint32_t)_cbor_remaining(enc) < len) {
    return CBOR_ERR_EOF;
  }
  if (res > 0) {
    memcpy(enc->curr, buf, len);
    enc->curr += len;
  }
  return res;
}
cbor_result_t cbor_encode_str(cbor_value_t *enc, const char *buf) {
  uint32_t len = strlen(buf);
  return cbor_encode_tstr(enc, (uint8_t *)buf, len);
}

cbor_result_t cbor_encode_tag(cbor_value_t *dec, const uint32_t *val) {
  return _cbor_encode_raw(dec, CBOR_TYPE_TAG, (const uint8_t *)val, _cbor_size_for_value(*val));
}