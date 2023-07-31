#include "driver/gpio.h"

#include "core/project.h"

void gpio_ports_init() {
}

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config) {
}

void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af) {
}

void gpio_pin_set(gpio_pins_t pin) {
}

void gpio_pin_reset(gpio_pins_t pin) {
}

void gpio_pin_toggle(gpio_pins_t pin) {
}

uint32_t gpio_pin_read(gpio_pins_t pin) {
  return 1;
}

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(port_num, num) \
  {                             \
      .port = GPIO##port_num,   \
      .pin_index = num,         \
      .pin = GPIO_PINS_##num,   \
  },

const gpio_pin_def_t gpio_pin_defs[PINS_MAX] = {
    {},
#include "gpio_pins.in"
};

#undef GPIO_PIN
#undef GPIO_AF

#define GPIO_AF(_pin, _af, _tag) \
  {                              \
      .pin = _pin,               \
      .tag = _tag,               \
      .af = _af,                 \
  },
#define GPIO_PIN(port_num, num)

const gpio_af_t gpio_pin_afs[] = {
#include "gpio_pins.in"
};

#undef GPIO_PIN
#undef GPIO_AF

const uint32_t GPIO_AF_MAX = (sizeof(gpio_pin_afs) / sizeof(gpio_af_t));
