#include "driver/gpio.h"

#include "core/project.h"
#include "driver/adc.h"
#include "driver/timer.h"

static constexpr gpio_drive_strength drive_map[] = {
    GPIO_DRIVE_STRENGTH_4MA,  // GPIO_DRIVE_NORMAL
    GPIO_DRIVE_STRENGTH_12MA, // GPIO_DRIVE_HIGH
};

uint64_t gpio_open_drain_mask = 0;

void gpio_ports_init() {
  unreset_block_wait(RESETS_RESET_IO_BANK0_BITS | RESETS_RESET_PADS_BANK0_BITS);
}

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config) {
  const uint32_t index = gpio_pin_defs[pin].pin_index;
  const uint64_t bit = 1ull << index;

  gpio_set_pulls(index, config.pull == GPIO_UP_PULL, config.pull == GPIO_DOWN_PULL);
  gpio_set_drive_strength(index, drive_map[config.drive]);

  const bool open_drain = config.mode == GPIO_OUTPUT && config.output == GPIO_OPENDRAIN;
  if (open_drain)
    gpio_open_drain_mask |= bit;
  else
    gpio_open_drain_mask &= ~bit;

  switch (config.mode) {
  case GPIO_OUTPUT:
    gpio_init(index);
    if (open_drain) {
      gpio_put(index, false);
      gpio_set_dir(index, false);
    } else {
      gpio_set_dir(index, true);
    }
    break;
  case GPIO_INPUT:
    gpio_init(index);
    gpio_set_dir(index, false);
    break;
  case GPIO_ANALOG:
    gpio_set_function(index, GPIO_FUNC_NULL);
    gpio_set_input_enabled(index, false);
    break;
  case GPIO_ALTERNATE:
    // gpio_pin_init_af() selects the function; peripherals that need an
    // open-drain bus (I2C) only ever drive it low.
    gpio_set_input_enabled(index, true);
    break;
  }
}

// Outputs and inputs keep the SIO function; I2C unstick drives its pins as
// open-drain outputs through their tags before handing them to the block.
void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af) {
  gpio_pin_init(pin, config);
  if (config.mode == GPIO_ALTERNATE)
    gpio_set_function(gpio_pin_defs[pin].pin_index, static_cast<gpio_function_t>(af));
}

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(port_num, num)  \
  {                              \
      .port = sio_hw,            \
      .pin_index = num,          \
      .pin = 1u << ((num) % 32), \
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
