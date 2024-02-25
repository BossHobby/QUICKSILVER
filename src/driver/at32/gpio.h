#pragma once

#define gpio_pin_set(_pin) gpio_bits_set(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)
#define gpio_pin_reset(_pin) gpio_bits_reset(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)
#define gpio_pin_toggle(_pin) (gpio_output_data_bit_read(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin) ? gpio_pin_reset(_pin) : gpio_pin_set(_pin))
#define gpio_pin_read(_pin) gpio_input_data_bit_read(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)