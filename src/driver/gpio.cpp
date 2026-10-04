#include "driver/gpio.h"

bool gpio_pin_has_tag(gpio_pins_t pin, resource_tag_t tag) {
  for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
    if (gpio_pin_afs[i].pin == pin && gpio_pin_afs[i].tag == tag)
      return true;
  }
  return false;
}

void gpio_pin_init_tag(gpio_pins_t pin, gpio_config_t config, resource_tag_t tag) {
  for (uint32_t j = 0; j < GPIO_AF_MAX; j++) {
    const gpio_af_t *func = &gpio_pin_afs[j];
    if (func->pin != pin || func->tag != tag) {
      continue;
    }
    return gpio_pin_init_af(pin, config, func->af);
  }
}
