#pragma once

#include <stdbool.h>

#include <cbor.h>

#include "gpio_pins.h"

#define LED_MAX 4

typedef struct {
  gpio_pins_t pin;
  bool invert;
} target_led_t;

#define TARGET_LED_MEMBERS \
  MEMBER(pin, gpio_pins_t) \
  MEMBER(invert, bool)

typedef struct {
  uint8_t name[32];

  target_led_t leds[LED_MAX];
} target_t;

#define TARGET_MEMBERS  \
  TSTR_MEMBER(name, 32) \
  ARRAY_MEMBER(leds, LED_MAX, target_led_t)

extern target_t target;

cbor_result_t cbor_encode_gpio_pins_t(cbor_value_t *enc, const gpio_pins_t *t);
cbor_result_t cbor_decode_gpio_pins_t(cbor_value_t *dec, gpio_pins_t *t);

cbor_result_t cbor_encode_target_t(cbor_value_t *enc, const target_t *t);
cbor_result_t cbor_decode_target_t(cbor_value_t *dec, target_t *t);