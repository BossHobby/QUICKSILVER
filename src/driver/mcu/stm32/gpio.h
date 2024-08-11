#pragma once

#define gpio_pin_set(_pin) LL_GPIO_SetOutputPin(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)
#define gpio_pin_reset(_pin) LL_GPIO_ResetOutputPin(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)
#define gpio_pin_toggle(_pin) LL_GPIO_TogglePin(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)
#define gpio_pin_read(_pin) LL_GPIO_IsInputPinSet(gpio_pin_defs[_pin].port, gpio_pin_defs[_pin].pin)