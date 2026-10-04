#include <string.h>
#include <unity.h>

#include "control/control.h"
#include "core/profile.h"
#include "driver/gpio.h"
#include "io/pinio.h"

static void reset_pinio() {
  target = {};
  state = {};
  flags = {};
}

void test_pinio_polarity_defaults_and_independent_switches() {
  reset_pinio();
  target.pinio[0] = {.pin = PIN_B0};
  target.pinio[1] = {.pin = PIN_B1, .invert = true};
  target.pinio[2] = {.pin = PIN_B2};
  target.pinio[3] = {.pin = PIN_B3, .invert = true};
  profile_set_defaults(&profile);
  TEST_ASSERT_EQUAL(RX_CHANNEL_OFF, profile.receiver.aux[AUX_PINIO_1].channel);
  TEST_ASSERT_EQUAL(RX_CHANNEL_OFF, profile.receiver.aux[AUX_PINIO_2].channel);
  profile.receiver.aux[AUX_PINIO_1].channel = RX_CHANNEL_ON;
  profile.receiver.aux[AUX_PINIO_4].channel = RX_CHANNEL_ON;
  pinio_init();
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B1));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B2));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B3));

  flags.rx_mode = RXMODE_NORMAL;
  flags.arm_state = 1;
  flags.in_air = 1;
  state.aux_active = (1u << AUX_PINIO_2) | (1u << AUX_PINIO_3);
  TEST_ASSERT_EQUAL(pdMS_TO_TICKS(10), pinio_update());
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B1));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B2));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B3));
  flags = {};
  target = {};
}

void test_pinio_target_changes_require_reinitialization() {
  reset_pinio();
  target.pinio[0] = {.pin = PIN_B0};
  profile_set_defaults(&profile);
  profile.receiver.aux[AUX_PINIO_1].channel = RX_CHANNEL_ON;
  pinio_init();
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B0));
  target.pinio[0] = {.pin = PIN_B1, .invert = true};
  gpio_pin_set(PIN_B1);
  pinio_update();
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B1));
  pinio_init();
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B1));
  flags = {};
  target = {};
}

void test_pinio_defers_swd_until_switched_on() {
  reset_pinio();
  target.pinio[0] = {.pin = PIN_A13, .invert = true};
  target.pinio[1] = {.pin = PIN_B0};
  profile_set_defaults(&profile);
  profile.receiver.aux[AUX_PINIO_2] = {RX_CHANNEL_5, AUX_VALUE_MID, AUX_VALUE_MAX};
  gpio_pin_set(PIN_A13);
  pinio_init();
  state.aux_active = 1u << AUX_PINIO_1;
  pinio_update();
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_A13));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  state.aux_active = 0;
  flags.rx_mode = RXMODE_NORMAL;
  pinio_update();
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_A13));
  state.aux_active = 1u << AUX_PINIO_1;
  pinio_update();
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_A13));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  target = {};
  pinio_init();
  TEST_ASSERT_EQUAL(portMAX_DELAY, pinio_update());
  flags = {};
}

void test_pinio_target_cbor_roundtrip() {
  target_t source = {};
  source.pinio[0] = {.pin = PIN_B0, .invert = true, .label = "VTX power", .description = "Video transmitter supply"};
  source.pinio[1] = {.pin = PIN_B1, .label = "Camera select"};
  uint8_t buffer[4096];
  cbor_value_t codec;
  cbor_encoder_init(&codec, buffer, sizeof(buffer));
  TEST_ASSERT_TRUE(cbor_encode_target_t(&codec, &source) >= CBOR_OK);
  const uint32_t length = cbor_encoder_len(&codec);
  target_t decoded = {};
  cbor_decoder_init(&codec, buffer, length);
  TEST_ASSERT_TRUE(cbor_decode_target_t(&codec, &decoded) >= CBOR_OK);
  TEST_ASSERT_EQUAL(PIN_B0, decoded.pinio[0].pin);
  TEST_ASSERT_TRUE(decoded.pinio[0].invert);
  TEST_ASSERT_EQUAL_STRING("VTX power", decoded.pinio[0].label);
  TEST_ASSERT_EQUAL_STRING("Video transmitter supply", decoded.pinio[0].description);
  TEST_ASSERT_EQUAL(PIN_B1, decoded.pinio[1].pin);
  TEST_ASSERT_FALSE(decoded.pinio[1].invert);
  TEST_ASSERT_EQUAL(PIN_NONE, decoded.pinio[2].pin);
}

void test_aux_output_constants_apply_without_receiver_frames() {
  reset_pinio();
  target.pinio[0] = {.pin = PIN_B0};
  target.pinio[1] = {.pin = PIN_B1, .invert = true};
  target.pinio[2] = {.pin = PIN_B2};
  profile_set_defaults(&profile);
  profile.receiver.protocol = RX_PROTOCOL_INVALID;
  profile.receiver.aux[AUX_PINIO_3] = {RX_CHANNEL_5, AUX_VALUE_MID, AUX_VALUE_MAX};
  state.rx_filter_hz = 20.0f;
  state.looptime_autodetect = 125.0f;
  rx_init();
  pinio_init();
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B1));
  TEST_ASSERT_FALSE(rx_aux_on(AUX_VTX_PIT_MODE));
  TEST_ASSERT_FALSE(rx_aux_on(AUX_BUZZER_ENABLE));

  profile.receiver.aux[AUX_VTX_PIT_MODE].channel = RX_CHANNEL_ON;
  profile.receiver.aux[AUX_BUZZER_ENABLE].channel = RX_CHANNEL_ON;
  profile.receiver.aux[AUX_PINIO_1].channel = RX_CHANNEL_ON;
  profile.receiver.aux[AUX_PINIO_2].channel = RX_CHANNEL_ON;
  rx_process();
  pinio_update();
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B1));
  TEST_ASSERT_TRUE(rx_aux_on(AUX_VTX_PIT_MODE));
  TEST_ASSERT_TRUE(rx_aux_on(AUX_BUZZER_ENABLE));

  // Loss of RX preserves the last channel-controlled output while constant
  // assignments can still be changed through ground configuration.
  flags.failsafe_signal_lost = 1;
  state.aux_active |= 1U << AUX_PINIO_3;
  profile.receiver.aux[AUX_VTX_PIT_MODE].channel = RX_CHANNEL_OFF;
  profile.receiver.aux[AUX_BUZZER_ENABLE].channel = RX_CHANNEL_OFF;
  profile.receiver.aux[AUX_PINIO_1].channel = RX_CHANNEL_OFF;
  profile.receiver.aux[AUX_PINIO_2].channel = RX_CHANNEL_OFF;
  rx_process();
  pinio_update();
  TEST_ASSERT_FALSE(gpio_pin_read(PIN_B0));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B1));
  TEST_ASSERT_TRUE(gpio_pin_read(PIN_B2));
  TEST_ASSERT_FALSE(rx_aux_on(AUX_VTX_PIT_MODE));
  TEST_ASSERT_FALSE(rx_aux_on(AUX_BUZZER_ENABLE));
  flags = {};
  target = {};
}
