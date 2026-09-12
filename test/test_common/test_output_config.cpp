#include <string.h>
#include <unity.h>

#include "core/profile.h"

void test_output_protocol_snapshot_updates_explicitly() {
  const profile_t saved_profile = profile;
  memset(profile.outputs, 0, sizeof(profile.outputs));
  profile.outputs[0].target_output = 6;
  profile.outputs[0].protocol = OUTPUT_PROTOCOL_DSHOT;
  profile.outputs[1].target_output = 2;
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
  TEST_ASSERT_TRUE(profile_output_slot_uses_protocol(2, OUTPUT_PROTOCOL_PWM));

  memset(profile.outputs, 0, sizeof(profile.outputs));
  TEST_ASSERT_TRUE(profile_output_slot_uses_protocol(6, OUTPUT_PROTOCOL_DSHOT));
  profile_output_update();
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_DSHOT));
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(OUTPUT_PROTOCOL_PWM));
  TEST_ASSERT_FALSE(profile_output_slot_uses_protocol(2, OUTPUT_PROTOCOL_PWM));
  TEST_ASSERT_FALSE(profile_outputs_use_protocol(static_cast<output_protocol_t>(255)));

  profile = saved_profile;
  profile_output_update();
}
