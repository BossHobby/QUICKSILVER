#include "driver/gpio.h"

// init fpv pin separately because it may use SWDAT/SWCLK don't want to enable it right away
bool gpio_init_fpv(uint8_t mode) {
  static volatile uint8_t fpv_init_done = 0;
  if (target.fpv == PIN_NONE) {
    return false;
  }

  // only repurpose the pin after rx/tx have bound if it is swd
  // common settings to set ports
  if (mode == 1 && fpv_init_done == 0) {
    // set gpio pin as output no matter what
    gpio_config_t init;
    init.mode = GPIO_OUTPUT;
    init.output = GPIO_PUSHPULL;
    init.pull = GPIO_NO_PULL;
    init.drive = GPIO_DRIVE_HIGH;
    gpio_pin_init(target.fpv, init);
    return true;
  }

  return mode == 1 && fpv_init_done == 1;
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