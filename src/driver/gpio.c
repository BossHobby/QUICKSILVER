#include "driver/gpio.h"

#include "core/project.h"
#include "driver/rcc.h"

static volatile uint8_t fpv_init_done = 0;

void gpio_init() {
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

#if defined(FPV_PIN)
  // skip repurpose of swd pin @boot
  if (FPV_PIN != PIN_A13 && FPV_PIN != PIN_A14) {
    LL_GPIO_InitTypeDef init;
    init.Mode = LL_GPIO_MODE_OUTPUT;
    init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    init.Pull = LL_GPIO_PULL_NO;
    init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    gpio_pin_init(&init, FPV_PIN);
    gpio_pin_reset(FPV_PIN);
    fpv_init_done = 1;
  }
#endif
}

// init fpv pin separately because it may use SWDAT/SWCLK don't want to enable it right away
int gpio_init_fpv(uint8_t mode) {
#if defined(FPV_PIN)
  // only repurpose the pin after rx/tx have bound if it is swd
  // common settings to set ports
  LL_GPIO_InitTypeDef init;
  init.Mode = LL_GPIO_MODE_OUTPUT;
  init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  init.Pull = LL_GPIO_PULL_NO;
  init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  if (mode == 1 && fpv_init_done == 0) {
    // set gpio pin as output no matter what
    gpio_pin_init(&init, FPV_PIN);
    return 1;
  }
  if (mode == 1 && fpv_init_done == 1) {
    return 1;
  }
#endif
  return 0;
}

void gpio_pin_init(LL_GPIO_InitTypeDef *init, gpio_pins_t pin) {
  init->Pin = gpio_pin_defs[pin].pin;
  LL_GPIO_Init(gpio_pin_defs[pin].port, init);
}

void gpio_pin_init_af(LL_GPIO_InitTypeDef *init, gpio_pins_t pin, uint32_t af) {
  init->Alternate = af;
  gpio_pin_init(init, pin);
}

void gpio_pin_set(gpio_pins_t pin) {
  LL_GPIO_SetOutputPin(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

void gpio_pin_reset(gpio_pins_t pin) {
  LL_GPIO_ResetOutputPin(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

void gpio_pin_toggle(gpio_pins_t pin) {
  LL_GPIO_TogglePin(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

uint32_t gpio_pin_read(gpio_pins_t pin) {
  return LL_GPIO_IsInputPinSet(gpio_pin_defs[pin].port, gpio_pin_defs[pin].pin);
}

#define GPIO_PIN(port_num, num) MAKE_PIN_DEF(port_num, num),

const gpio_pin_def_t gpio_pin_defs[PINS_MAX] = {
    {},
#include "config/gpio_pins.in"
};

#undef GPIO_PIN
