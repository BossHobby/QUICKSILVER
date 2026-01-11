#include "driver/gpio.h"

#include "core/project.h"

// Define virtual GPIO port constants for native
static uint32_t GPIO_PORT_A = 0;
static uint32_t GPIO_PORT_B = 1;
static uint32_t GPIO_PORT_C = 2;
static uint32_t GPIO_PORT_D = 3;

#define GPIOA (&GPIO_PORT_A)
#define GPIOB (&GPIO_PORT_B)
#define GPIOC (&GPIO_PORT_C)
#define GPIOD (&GPIO_PORT_D)

// Define virtual GPIO pin constants for native
#define GPIO_PINS_0  (1 << 0)
#define GPIO_PINS_1  (1 << 1)
#define GPIO_PINS_2  (1 << 2)
#define GPIO_PINS_3  (1 << 3)
#define GPIO_PINS_4  (1 << 4)
#define GPIO_PINS_5  (1 << 5)
#define GPIO_PINS_6  (1 << 6)
#define GPIO_PINS_7  (1 << 7)
#define GPIO_PINS_8  (1 << 8)
#define GPIO_PINS_9  (1 << 9)
#define GPIO_PINS_10 (1 << 10)
#define GPIO_PINS_11 (1 << 11)
#define GPIO_PINS_12 (1 << 12)
#define GPIO_PINS_13 (1 << 13)
#define GPIO_PINS_14 (1 << 14)
#define GPIO_PINS_15 (1 << 15)

void gpio_ports_init() {
}

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config) {
}

void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af) {
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