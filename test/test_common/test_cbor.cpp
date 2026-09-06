#include <unity.h>

#include "core/profile.h"
#include "util/cbor_helper.h"

// Both ordinary members and enum storage must match the selected wire codec.
template <typename T, typename Wire>
concept cbor_codec_matches = requires(cbor_value_t *codec, T *value,
                                      cbor_result_t (*encode)(cbor_value_t *, const Wire *),
                                      cbor_result_t (*decode)(cbor_value_t *, Wire *)) {
  cbor_encode_member(codec, value, encode);
  cbor_decode_member(codec, value, decode);
};

static_assert(cbor_codec_matches<uint8_t, uint8_t>);
static_assert(cbor_codec_matches<dshot_time_t, uint16_t>);
static_assert(cbor_codec_matches<rx_channel_t, uint8_t>);
static_assert(!cbor_codec_matches<uint16_t, uint8_t>);
static_assert(!cbor_codec_matches<float, uint32_t>);
static_assert(!cbor_codec_matches<int8_t, uint8_t>);
static_assert(!cbor_codec_matches<dshot_time_t, uint8_t>);
static_assert(!cbor_codec_matches<rx_channel_t, uint16_t>);

struct cbor_test_settings_t {
  dshot_time_t rate;
  rx_channel_t channels[2];
  uint8_t guard;
};

CBOR_START_STRUCT_ENCODER(cbor_test_settings_t)
CBOR_ENCODE_MEMBER(rate, uint16_t)
CBOR_ENCODE_ARRAY_MEMBER(channels, 2, uint8_t)
CBOR_ENCODE_MEMBER(guard, uint8_t)
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_DECODER(cbor_test_settings_t)
CBOR_DECODE_MEMBER(rate, uint16_t)
CBOR_DECODE_ARRAY_MEMBER(channels, 2, uint8_t)
CBOR_DECODE_MEMBER(guard, uint8_t)
CBOR_END_STRUCT_DECODER()

void test_cbor_enum_wire_format(void) {
  const cbor_test_settings_t settings = {DSHOT_TIME_600, {RX_CHANNEL_1, RX_CHANNEL_2}, 0xa5};
  uint8_t buffer[64] = {};
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_cbor_test_settings_t(&codec, &settings) >= CBOR_OK);

  const uint8_t expected[] = {
      0xbf,
      0x64,
      'r',
      'a',
      't',
      'e',
      0x19,
      0x02,
      0x58,
      0x68,
      'c',
      'h',
      'a',
      'n',
      'n',
      'e',
      'l',
      's',
      0x82,
      0x00,
      0x01,
      0x65,
      'g',
      'u',
      'a',
      'r',
      'd',
      0x18,
      0xa5,
      0xff,
  };
  TEST_ASSERT_EQUAL_UINT32(sizeof(expected), cbor_encoder_len(&codec));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, buffer, sizeof(expected));

  cbor_test_settings_t decoded = {};
  cbor_decoder_init(&codec, buffer, sizeof(expected));
  TEST_ASSERT_TRUE(cbor_decode_cbor_test_settings_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL(DSHOT_TIME_600, decoded.rate);
  TEST_ASSERT_EQUAL(RX_CHANNEL_1, decoded.channels[0]);
  TEST_ASSERT_EQUAL(RX_CHANNEL_2, decoded.channels[1]);
  TEST_ASSERT_EQUAL_HEX8(0xa5, decoded.guard);
}

void test_cbor_failed_enum_decode_preserves_value(void) {
  // The integer decoder clears its destination before detecting a truncated value.
  uint8_t truncated[] = {0x19, 0x02};
  dshot_time_t rate = DSHOT_TIME_600;
  cbor_value_t codec;
  cbor_decoder_init(&codec, truncated, sizeof(truncated));
  TEST_ASSERT_EQUAL(CBOR_ERR_EOF, cbor_decode_member(&codec, &rate, cbor_decode_uint16_t));
  TEST_ASSERT_EQUAL(DSHOT_TIME_600, rate);

  uint8_t overflow[] = {0x19, 0x01, 0x00};
  rx_channel_t channel = RX_CHANNEL_2;
  cbor_decoder_init(&codec, overflow, sizeof(overflow));
  TEST_ASSERT_EQUAL(CBOR_ERR_OVERFLOW, cbor_decode_member(&codec, &channel, cbor_decode_uint8_t));
  TEST_ASSERT_EQUAL(RX_CHANNEL_2, channel);
}
