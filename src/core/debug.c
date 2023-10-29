#include "core/debug.h"

#include "driver/gpio.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/cbor_helper.h"

#ifdef DEBUG

void debug_pin_init() {
#if defined(DEBUG_PIN0) || defined(DEBUG_PIN1)
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.pull = GPIO_NO_PULL;
#endif

#ifdef DEBUG_PIN0
  gpio_pin_init(DEBUG_PIN0, gpio_init);
#endif

#ifdef DEBUG_PIN1
  gpio_pin_init(DEBUG_PIN1, gpio_init);
#endif
}

void debug_pin_enable(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_set(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_set(DEBUG_PIN1);
  }
#endif
}

void debug_pin_disable(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_reset(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_reset(DEBUG_PIN1);
  }
#endif
}

void debug_pin_toggle(uint8_t index) {
#ifdef DEBUG_PIN0
  if (index == 0) {
    gpio_pin_toggle(DEBUG_PIN0);
  }
#endif

#ifdef DEBUG_PIN1
  if (index == 1) {
    gpio_pin_toggle(DEBUG_PIN1);
  }
#endif
}

#else

void debug_pin_init() {}

void debug_pin_enable(uint8_t index) {}
void debug_pin_disable(uint8_t index) {}
void debug_pin_toggle(uint8_t index) {}

#endif
