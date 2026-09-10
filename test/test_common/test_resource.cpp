#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "driver/resource.h"
#include "driver/timer.h"

void test_resource_timer_tag_roundtrip() {
  const struct {
    timer_index_t index;
    unsigned number;
  } timers[] = {
#define TIMER(num) {TIMER##num, num},
      TIMERS
#undef TIMER
  };
  const timer_channel_t channels[] = {TIMER_CH1, TIMER_CH2, TIMER_CH3, TIMER_CH4};

  for (const auto &timer : timers) {
    for (unsigned ch = 0; ch < 4; ch++) {
      const resource_tag_t tag = TIMER_TAG(timer.index, channels[ch]);
      uint8_t buffer[64];
      cbor_value_t codec;
      cbor_encoder_init(&codec, buffer, sizeof(buffer));
      TEST_ASSERT_TRUE(cbor_encode_resource_tag_t(&codec, &tag) >= CBOR_OK);
      const uint32_t size = cbor_encoder_len(&codec);

      cbor_decoder_init(&codec, buffer, size);
      const uint8_t *name;
      uint32_t length;
      TEST_ASSERT_TRUE(cbor_decode_tstr(&codec, &name, &length) >= CBOR_OK);
      char expected[32];
      snprintf(expected, sizeof(expected), "TIMER%u_CH%u", timer.number, ch + 1);
      TEST_ASSERT_EQUAL_UINT32(strlen(expected), length);
      TEST_ASSERT_EQUAL_MEMORY(expected, name, length);

      cbor_decoder_init(&codec, buffer, size);
      resource_tag_t restored = RESOURCE_INVALID;
      TEST_ASSERT_TRUE(cbor_decode_resource_tag_t(&codec, &restored) >= CBOR_OK);
      TEST_ASSERT_EQUAL_UINT32(tag, restored);
    }
  }
}
