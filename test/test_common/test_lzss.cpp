#include <string.h>
#include <unity.h>

#include "core/profile.h"
#include "io/quic.h"
#include "util/cbor_helper.h"
#include "util/lzss.h"

extern const profile_t default_profile;

// Reference decoder matching the Configurator's.
static uint32_t lzss_decode(const uint8_t *in, uint32_t in_size, uint8_t *out, uint32_t out_size) {
  uint32_t i = 0;
  uint32_t o = 0;
  while (i < in_size) {
    const uint8_t control = in[i++];
    for (uint32_t k = 0; k < 8 && i < in_size; k++) {
      if (!(control & (1 << k))) {
        if (o >= out_size)
          return 0;
        out[o++] = in[i++];
        continue;
      }
      if (i + 2 > in_size)
        return 0;
      const uint16_t token = in[i] << 8 | in[i + 1];
      i += 2;
      const uint32_t dist = (token >> LZSS_LENGTH_BITS) + 1;
      const uint32_t len = (token & ((1 << LZSS_LENGTH_BITS) - 1)) + LZSS_MIN_MATCH;
      if (dist > o || o + len > out_size)
        return 0;
      for (uint32_t n = 0; n < len; n++, o++)
        out[o] = out[o - dist];
    }
  }
  return o;
}

static uint8_t compressed[8192];
static uint32_t compressed_size;

static void collect(const uint8_t *data, uint32_t len, void *) {
  TEST_ASSERT_TRUE(compressed_size + len <= sizeof(compressed));
  memcpy(compressed + compressed_size, data, len);
  compressed_size += len;
}

static void assert_roundtrip(const uint8_t *data, uint32_t size) {
  static uint16_t table[LZSS_HASH_SIZE];
  static uint8_t decompressed[8192];
  compressed_size = 0;
  const uint32_t expected_size = lzss_encode(data, size, table, NULL, NULL);
  TEST_ASSERT_EQUAL_UINT32(expected_size, lzss_encode(data, size, table, collect, NULL));
  TEST_ASSERT_EQUAL_UINT32(expected_size, compressed_size);
  TEST_ASSERT_EQUAL_UINT32(size, lzss_decode(compressed, compressed_size, decompressed, sizeof(decompressed)));
  TEST_ASSERT_EQUAL_MEMORY(data, decompressed, size);
}

static uint32_t pseudo_random(uint32_t *seed) {
  *seed = *seed * 1664525 + 1013904223;
  return *seed >> 24;
}

void test_lzss_roundtrip(void) {
  static uint8_t data[6000];

  cbor_value_t enc;
  cbor_encoder_init(&enc, data, sizeof(data));
  TEST_ASSERT_TRUE(cbor_encode_profile_t(&enc, &default_profile) >= CBOR_OK);
  const uint32_t profile_size = cbor_encoder_len(&enc);
  assert_roundtrip(data, profile_size);

  // Runs overlap their own source and exceed the longest match.
  memset(data, 'a', 1000);
  assert_roundtrip(data, 1000);

  // Repeats exactly one window apart and just beyond it.
  uint32_t seed = 1;
  for (uint32_t i = 0; i < LZSS_WINDOW; i++)
    data[i] = pseudo_random(&seed);
  memcpy(data + LZSS_WINDOW, data, LZSS_WINDOW);
  memcpy(data + 2 * LZSS_WINDOW + 1, data + 1, 100);
  assert_roundtrip(data, 2 * LZSS_WINDOW + 101);
}

static uint8_t reply[4200];
static uint32_t reply_size;

// Compressed replies arrive as a header followed by chunks.
static void capture_reply(uint8_t *data, uint32_t size, void *) {
  TEST_ASSERT_TRUE(reply_size + size <= sizeof(reply));
  memcpy(reply + reply_size, data, size);
  reply_size += size;
}

void test_quic_compresses_reply_only_when_requested(void) {
  profile = default_profile;
  quic_t quic = {.send = capture_reply};

  uint8_t request[] = {QUIC_MAGIC, QUIC_CMD_GET, 0, 1, QUIC_VAL_PROFILE};
  reply_size = 0;
  TEST_ASSERT_TRUE(quic_process(&quic, request, sizeof(request)));
  TEST_ASSERT_EQUAL_HEX8(QUIC_CMD_GET, reply[1]);
  static uint8_t raw[sizeof(reply)];
  const uint32_t raw_size = reply_size - QUIC_HEADER_LEN;
  TEST_ASSERT_EQUAL_UINT32(raw_size, reply[2] << 8 | reply[3]);
  memcpy(raw, reply + QUIC_HEADER_LEN, raw_size);

  request[1] |= QUIC_HEADER_COMPRESSED;
  reply_size = 0;
  TEST_ASSERT_TRUE(quic_process(&quic, request, sizeof(request)));
  TEST_ASSERT_EQUAL_HEX8(QUIC_CMD_GET | QUIC_HEADER_COMPRESSED, reply[1]);
  const uint32_t payload_size = reply_size - QUIC_HEADER_LEN;
  TEST_ASSERT_EQUAL_UINT32(payload_size, reply[2] << 8 | reply[3]);
  TEST_ASSERT_TRUE(payload_size < raw_size);

  static uint8_t decompressed[sizeof(reply)];
  TEST_ASSERT_EQUAL_UINT32(raw_size, lzss_decode(reply + QUIC_HEADER_LEN, payload_size, decompressed, sizeof(decompressed)));
  TEST_ASSERT_EQUAL_MEMORY(raw, decompressed, raw_size);

  // Errors stay uncompressed so any client can read them.
  uint8_t invalid[] = {QUIC_MAGIC, (uint8_t)(QUIC_CMD_GET | QUIC_HEADER_COMPRESSED), 0, 1, 23};
  reply_size = 0;
  TEST_ASSERT_TRUE(quic_process(&quic, invalid, sizeof(invalid)));
  TEST_ASSERT_EQUAL_HEX8(QUIC_CMD_GET | QUIC_FLAG_ERROR << QUIC_HEADER_FLAG_SHIFT, reply[1]);
}
