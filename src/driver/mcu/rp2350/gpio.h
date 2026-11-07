#pragma once

#include <stdint.h>

// Pads have no open-drain mode. Pins configured as open-drain outputs hold
// their output low and toggle the output enable: set releases the pin to its
// pull-up, reset drives it low.
extern uint64_t gpio_open_drain_mask;

static inline void gpio_pin_write(uint32_t index, bool value) {
  if ((gpio_open_drain_mask >> index) & 1)
    gpio_set_dir(index, !value);
  else
    gpio_put(index, value);
}

static inline bool gpio_pin_level(uint32_t index) {
  if ((gpio_open_drain_mask >> index) & 1)
    return !gpio_is_dir_out(index);
  return gpio_get_out_level(index);
}

#define gpio_pin_set(_pin) gpio_pin_write(gpio_pin_defs[_pin].pin_index, true)
#define gpio_pin_reset(_pin) gpio_pin_write(gpio_pin_defs[_pin].pin_index, false)
#define gpio_pin_toggle(_pin) gpio_pin_write(gpio_pin_defs[_pin].pin_index, !gpio_pin_level(gpio_pin_defs[_pin].pin_index))
#define gpio_pin_read(_pin) gpio_get(gpio_pin_defs[_pin].pin_index)
