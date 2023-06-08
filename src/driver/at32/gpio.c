#include "driver/gpio.h"

#include "core/project.h"
#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/timer.h"

static const uint32_t mode_map[] = {
    [GPIO_INPUT] = GPIO_MODE_INPUT,
    [GPIO_OUTPUT] = GPIO_MODE_OUTPUT,
    [GPIO_ANALOG] = GPIO_MODE_ANALOG,
    [GPIO_ALTERNATE] = GPIO_MODE_MUX,
};

static const uint32_t output_map[] = {
    [GPIO_PUSHPULL] = GPIO_OUTPUT_PUSH_PULL,
    [GPIO_OPENDRAIN] = GPIO_OUTPUT_OPEN_DRAIN,
};

static const uint32_t speed_map[] = {
    [GPIO_DRIVE_NORMAL] = GPIO_DRIVE_STRENGTH_MODERATE,
    [GPIO_DRIVE_HIGH] = GPIO_DRIVE_STRENGTH_STRONGER,
};

static const uint32_t pull_map[] = {
    [GPIO_NO_PULL] = GPIO_PULL_NONE,
    [GPIO_UP_PULL] = GPIO_PULL_UP,
    [GPIO_DOWN_PULL] = GPIO_PULL_DOWN,
};

void gpio_ports_init() {
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOG_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOH_PERIPH_CLOCK, TRUE);
}

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config) {
  gpio_init_type init;
  init.gpio_mode = mode_map[config.mode];
  init.gpio_drive_strength = speed_map[config.drive];
  init.gpio_out_type = output_map[config.output];
  init.gpio_pull = pull_map[config.pull];
  init.gpio_pins = gpio_pin_defs[pin].pin;
  gpio_init(gpio_pin_defs[pin].port, &init);
}

void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af) {
  gpio_pin_init(pin, config);
  gpio_pin_mux_config(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin_index, af);
}

void gpio_pin_set(gpio_pins_t pin) {
  gpio_bits_set(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

void gpio_pin_reset(gpio_pins_t pin) {
  gpio_bits_reset(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

void gpio_pin_toggle(gpio_pins_t pin) {
  if (gpio_output_data_bit_read(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin)) {
    gpio_pin_reset(pin);
  } else {
    gpio_pin_set(pin);
  }
}

uint32_t gpio_pin_read(gpio_pins_t pin) {
  return gpio_input_data_bit_read(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
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
