#include "driver/gpio.h"

#include "core/project.h"
#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/timer.h"

static const uint32_t mode_map[] = {
    [GPIO_INPUT] = LL_GPIO_MODE_INPUT,
    [GPIO_OUTPUT] = LL_GPIO_MODE_OUTPUT,
    [GPIO_ANALOG] = LL_GPIO_MODE_ANALOG,
    [GPIO_ALTERNATE] = LL_GPIO_MODE_ALTERNATE,
};

static const uint32_t output_map[] = {
    [GPIO_PUSHPULL] = LL_GPIO_OUTPUT_PUSHPULL,
    [GPIO_OPENDRAIN] = LL_GPIO_OUTPUT_OPENDRAIN,
};

static const uint32_t speed_map[] = {
    [GPIO_DRIVE_NORMAL] = LL_GPIO_SPEED_FREQ_MEDIUM,
    [GPIO_DRIVE_HIGH] = LL_GPIO_SPEED_FREQ_HIGH,
};

static const uint32_t pull_map[] = {
    [GPIO_NO_PULL] = LL_GPIO_PULL_NO,
    [GPIO_UP_PULL] = LL_GPIO_PULL_UP,
    [GPIO_DOWN_PULL] = LL_GPIO_PULL_DOWN,
};

void gpio_ports_init() {
// clocks on to all ports
#ifdef STM32F4
  SET_BIT(
      RCC->AHB1ENR,
      RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
          RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOHEN);

  // for exit
  rcc_enable(RCC_APB2_GRP1(SYSCFG));
#endif

#ifdef STM32F7
  uint32_t ports = RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                   RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN |
                   RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_GPIOHEN | RCC_AHB1ENR_GPIOIEN;

#ifndef STM32F722
  ports |= RCC_AHB1ENR_GPIOJEN | RCC_AHB1ENR_GPIOKEN;
#endif

  SET_BIT(RCC->AHB1ENR, ports);
#endif

#ifdef STM32H7
  uint32_t ports = RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN |
                   RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN |
                   RCC_AHB4ENR_GPIOGEN | RCC_AHB4ENR_GPIOHEN | RCC_AHB4ENR_GPIOIEN |
                   RCC_AHB4ENR_GPIOJEN | RCC_AHB4ENR_GPIOKEN;

  SET_BIT(RCC->AHB4ENR, ports);
#endif
}

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config) {
  gpio_port_t *port = gpio_pin_defs[pin].port;
  const uint32_t pin_mask = gpio_pin_defs[pin].pin;

  if ((config.mode == GPIO_OUTPUT) || (config.mode == GPIO_ALTERNATE)) {
    LL_GPIO_SetPinSpeed(port, pin_mask, speed_map[config.drive]);
    LL_GPIO_SetPinOutputType(port, pin_mask, output_map[config.output]);
  }
  LL_GPIO_SetPinPull(port, pin_mask, pull_map[config.pull]);
  LL_GPIO_SetPinMode(port, pin_mask, mode_map[config.mode]);
}

void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af) {
  gpio_port_t *port = gpio_pin_defs[pin].port;
  const uint32_t pin_mask = gpio_pin_defs[pin].pin;

  if ((config.mode == GPIO_OUTPUT) || (config.mode == GPIO_ALTERNATE)) {
    LL_GPIO_SetPinSpeed(port, pin_mask, speed_map[config.drive]);
    LL_GPIO_SetPinOutputType(port, pin_mask, output_map[config.output]);
  }
  LL_GPIO_SetPinPull(port, pin_mask, pull_map[config.pull]);

  if (config.mode == GPIO_ALTERNATE) {
    if (POSITION_VAL(pin_mask) < 0x00000008U) {
      LL_GPIO_SetAFPin_0_7(port, pin_mask, af);
    } else {
      LL_GPIO_SetAFPin_8_15(port, pin_mask, af);
    }
  }

  LL_GPIO_SetPinMode(port, pin_mask, mode_map[config.mode]);
}

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(port_num, num) \
  {                             \
      .port = GPIO##port_num,   \
      .pin_index = num,         \
      .pin = LL_GPIO_PIN_##num, \
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
