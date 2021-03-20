#include "cbor.h"

#include <base64.h>
#include <cjson/cJSON.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void read_file(const char *filename, void **buf, uint32_t *len) {
  FILE *f = fopen(filename, "rb");
  fseek(f, 0, SEEK_END);
  *len = (uint32_t)ftell(f);

  fseek(f, 0, SEEK_SET);
  *buf = malloc(*len);
  fread(*buf, 1, *len, f);
  fclose(f);
}

typedef struct {
  uint8_t *cbor;
  int32_t cbor_len;
  cJSON *decoded;
} test_vector;

void read_json(const char *filename, test_vector **vectors, uint32_t *vectors_len) {
  uint32_t len;
  char *buf;
  read_file(filename, &buf, &len);

  cJSON *json = cJSON_Parse(buf);

  *vectors_len = (uint32_t)cJSON_GetArraySize(json);
  *vectors = malloc(*vectors_len * sizeof(test_vector));
  for (uint32_t i = 0; i < *vectors_len; i++) {
    cJSON *item = cJSON_GetArrayItem(json, i);

    cJSON *cbor = cJSON_GetObjectItem(item, "cbor");
    (*vectors)[i].cbor = unbase64(
        cbor->valuestring,
        (int)strlen(cbor->valuestring),
        &(*vectors)[i].cbor_len);

    (*vectors)[i].decoded = cJSON_GetObjectItem(item, "decoded");
  }

  free(buf);
}

cbor_result_t print_value(cbor_value_t *dec, cJSON *decoded) {
  int type = cbor_decode_type(dec);
  if (type < CBOR_OK) {
    return type;
  }
  switch (type) {
  case CBOR_TYPE_TAG: {
    uint32_t val = 0;
    cbor_result_t res = cbor_decode_tag(dec, &val);
    if (res < CBOR_OK) {
      printf("CBOR tag error %d\n", res);
      return res;
    }
    printf("CBOR tag %u\n", val);
    return res;
  }
  case CBOR_TYPE_UINT: {
    uint32_t val = 0;
    cbor_result_t res = cbor_decode_uint32(dec, &val);
    if (res < CBOR_OK) {
      printf("CBOR uint error %d\n", res);
      return res;
    }
    if (decoded && val != decoded->valuedouble) {
      printf("CBOR error mismatch val: %u expected: %u\n", val, (uint32_t)decoded->valuedouble);
    } else {
      printf("CBOR uint %u\n", val);
    }
    return res;
  }
  case CBOR_TYPE_NINT: {
    int32_t val = 0;
    cbor_result_t res = cbor_decode_int32(dec, &val);
    if (res < CBOR_OK) {
      printf("CBOR int error %d\n", res);
      return res;
    }
    if (decoded && val != decoded->valuedouble) {
      printf("CBOR error mismatch val: %d expected: %d\n", val, (int32_t)decoded->valuedouble);
    } else {
      printf("CBOR int %d\n", val);
    }
    return res;
  }
  case CBOR_TYPE_FLOAT: {
    float val = 0;
    cbor_result_t res = cbor_decode_float(dec, &val);
    if (res < CBOR_OK) {
      printf("CBOR float error %d\n", res);
      return res;
    }
    if (decoded && val != decoded->valuedouble) {
      printf("CBOR error mismatch val: %f expected: %f\n", val, decoded->valuedouble);
    } else {
      printf("CBOR float %f\n", val);
    }
    return res;
  }
  case CBOR_TYPE_BSTR: {
    uint32_t len;
    const uint8_t *val;
    cbor_result_t res = cbor_decode_bstr(dec, &val, &len);
    if (res < CBOR_OK) {
      printf("CBOR bstr error %d\n", res);
      return res;
    }
    printf("CBOR bstr %.*s (%d)\n", len, val, len);
    return res;
  }
  case CBOR_TYPE_TSTR: {
    uint64_t len;
    const uint8_t *val;
    cbor_result_t res = cbor_decode_tstr(dec, &val, &len);
    if (res < CBOR_OK) {
      printf("CBOR tstr error %d\n", res);
    } else {
      printf("CBOR tstr %.*s (%d)\n", len, val, len);
    }
    return res;
  }
  case CBOR_TYPE_ARRAY: {
    cbor_container_t array;
    cbor_result_t res = cbor_decode_array(dec, &array);
    if (res < CBOR_OK) {
      printf("CBOR array error %d\n", res);
      return res;
    }

    printf("CBOR array %d\n", res);
    for (int32_t j = 0; j < cbor_decode_array_size(dec, &array); j++) {
      int skip_type = cbor_decode_type(dec);
      printf("   ");
      cbor_result_t skip = print_value(dec, cJSON_GetArrayItem(decoded, j));
      if (skip < CBOR_OK) {
        printf("   CBOR array skip error %d type %d\n", skip, skip_type);
      }
    }

    return res;
  }
  case CBOR_TYPE_MAP: {
    cbor_container_t map;
    cbor_result_t res = cbor_decode_map(dec, &map);
    if (res < CBOR_OK) {
      printf("CBOR map error %d\n", res);
    } else {
      printf("CBOR map %d\n", res);

      for (int32_t j = 0; j < cbor_decode_map_size(dec, &map); j++) {
        int skip_key_type = cbor_decode_type(dec);
        printf("   ");
        cbor_result_t skip_key = print_value(dec, NULL);
        if (skip_key < CBOR_OK) {
          printf("  CBOR map skip_key error %d type %d\n", skip_key, skip_key_type);
        }

        int skip_value_type = cbor_decode_type(dec);
        printf("   ");
        cbor_result_t skip_value = print_value(dec, NULL);
        if (skip_value < CBOR_OK) {
          printf("  CBOR map skip_value error %d type %d\n", skip_value, skip_value_type);
        }
      }
    }
    return res;
  }
  default:
    printf("CBOR unknown type %d\n", type);
    return CBOR_OK;
  }
}

int main(int argc, char const *argv[]) {
  test_vector *vectors;
  uint32_t vectors_len;
  read_json("test/input/test_vectors.json", &vectors, &vectors_len);
  for (size_t i = 0; i < vectors_len; i++) {
    const test_vector *vec = &vectors[i];
    cbor_value_t dec;
    cbor_decoder_init(&dec, vec->cbor, vec->cbor_len);

    cbor_result_t res = CBOR_OK;
    while (res >= CBOR_OK) {
      res = print_value(&dec, vec->decoded);
    }
  }

  uint8_t buf[512];
  cbor_value_t enc;
  {
    cbor_encoder_init(&enc, buf, 512);
    cbor_encode_uint16(&enc, 1337);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);
    cbor_encode_int16(&enc, -1337);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);
    cbor_encode_float(&enc, -13.37f);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);
    cbor_encode_tstr(&enc, (uint8_t *)"HELLO", 5);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);

    cbor_encode_array(&enc, 3);
    cbor_encode_uint16(&enc, 1337);
    cbor_encode_int16(&enc, -1337);
    cbor_encode_float(&enc, -13.37f);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);

    cbor_encode_map(&enc, 3);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY1", 4);
    cbor_encode_uint16(&enc, 1337);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY2", 4);
    cbor_encode_int16(&enc, -1337);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY3", 4);
    cbor_encode_float(&enc, -13.37f);
    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }
  {
    cbor_encoder_init(&enc, buf, 512);

    cbor_encode_map_indefinite(&enc);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY1", 4);
    cbor_encode_uint16(&enc, 1337);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY2", 4);
    cbor_encode_int16(&enc, -1337);
    cbor_encode_tstr(&enc, (uint8_t *)"KEY3", 4);
    cbor_encode_float(&enc, -13.37f);
    cbor_encode_end_indefinite(&enc);

    uint32_t len = cbor_encoder_len(&enc);

    cbor_value_t dec;
    cbor_decoder_init(&dec, buf, len);
    print_value(&dec, NULL);
  }

  return 0;
}
