#include <string.h>
#include <unity.h>

#include "core/flash.h"
#include "core/profile.h"
#include "core/target.h"

void test_output_protocol_snapshot_updates_explicitly() {
  const profile_t saved_profile = profile;
  memset(profile.outputs, 0, sizeof(profile.outputs));
  profile.outputs[0].target_output = 6;
  profile.outputs[0].protocol = OUTPUT_PROTOCOL_DSHOT;
  profile.outputs[1].target_output = 12;
  profile.outputs[1].protocol = OUTPUT_PROTOCOL_PWM;
  profile.outputs[2].target_output = 6;
  profile.outputs[2].protocol = OUTPUT_PROTOCOL_DSHOT;
  profile.outputs[3].target_output = MOTOR_PIN_MAX;
  profile.outputs[3].protocol = OUTPUT_PROTOCOL_BRUSHED;
  profile_output_update();

  TEST_ASSERT_TRUE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT));
  TEST_ASSERT_TRUE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_BRUSHED));
  TEST_ASSERT_TRUE(profile_output_slot_uses_protocol(6, OUTPUT_PROTOCOL_DSHOT));
  TEST_ASSERT_FALSE(profile_output_slot_uses_protocol(0, OUTPUT_PROTOCOL_DSHOT));
  TEST_ASSERT_FALSE(profile_output_slot_uses_protocol(MOTOR_PIN_MAX, OUTPUT_PROTOCOL_BRUSHED));
  TEST_ASSERT_TRUE(profile_output_slot_uses_protocol(12, OUTPUT_PROTOCOL_PWM));
  TEST_ASSERT_FALSE(profile_output_slot_uses_protocol(4, OUTPUT_PROTOCOL_PWM));

  memset(profile.outputs, 0, sizeof(profile.outputs));
  TEST_ASSERT_TRUE(profile_output_slot_uses_protocol(6, OUTPUT_PROTOCOL_DSHOT));
  profile_output_update();
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT));
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_PWM));
  TEST_ASSERT_FALSE(profile_output_slot_uses_protocol(12, OUTPUT_PROTOCOL_PWM));
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(static_cast<output_protocol_t>(255)));

  profile = saved_profile;
  profile_output_update();
}

void test_target_decode_keeps_dshot_on_the_first_outputs() {
  target_t source = {};
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    source.outputs[i] = {.pin = (gpio_pins_t)(PIN_A0 + i), .caps = OUTPUT_CAP_PWM | OUTPUT_CAP_DSHOT};
  }

  uint8_t buffer[TARGET_STORAGE_SIZE];
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &source) >= CBOR_OK);

  target_t decoded = {};
  cbor_decoder_init(&codec, buffer, cbor_encoder_len(&codec));
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL(OUTPUT_CAP_PWM | OUTPUT_CAP_DSHOT, decoded.outputs[DSHOT_OUTPUT_MAX - 1].caps);
  TEST_ASSERT_EQUAL(OUTPUT_CAP_PWM, decoded.outputs[DSHOT_OUTPUT_MAX].caps);
  TEST_ASSERT_EQUAL(OUTPUT_CAP_PWM, decoded.outputs[MOTOR_PIN_MAX - 1].caps);
}
