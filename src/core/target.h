#pragma once

#include <cbor.h>
#include <stdbool.h>

#define LED_MAX 4

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(port, num) PIN_##port##num,
typedef enum {
  PIN_NONE,
#include "gpio_pins.in"
  PINS_MAX,
} gpio_pins_t;
#undef GPIO_PIN
#undef GPIO_AF

typedef enum {
  MOTOR_PIN0,
  MOTOR_PIN1,
  MOTOR_PIN2,
  MOTOR_PIN3,
  MOTOR_PIN_MAX
} motor_pin_t;

typedef enum {
  SERIAL_PORT_INVALID,
  SERIAL_PORT1,
  SERIAL_PORT2,
#if !defined(STM32F411)
  SERIAL_PORT3,
  SERIAL_PORT4,
  SERIAL_PORT5,
#endif
  SERIAL_PORT6,
#if defined(STM32F7) || defined(STM32H7)
  SERIAL_PORT7,
  SERIAL_PORT8,
#endif
  SERIAL_PORT_MAX,

  SERIAL_SOFT_INVALID = 100,
  SERIAL_SOFT_PORT1,
  SERIAL_SOFT_PORT2,
  SERIAL_SOFT_PORT3,
  SERIAL_SOFT_MAX,

  SERIAL_SOFT_START = SERIAL_SOFT_INVALID,
  SERIAL_SOFT_COUNT = SERIAL_SOFT_MAX - SERIAL_SOFT_START,
} serial_ports_t;

typedef struct {
  gpio_pins_t pin;
  bool invert;
} target_led_t;

#define TARGET_LED_MEMBERS \
  MEMBER(pin, gpio_pins_t) \
  MEMBER(invert, bool)

typedef struct {
  gpio_pins_t pin;
  bool invert;
} target_invert_pin_t;

#define TARGET_BUZZER_MEMBERS \
  MEMBER(pin, gpio_pins_t)    \
  MEMBER(invert, bool)

typedef struct {
  uint8_t index;
  gpio_pins_t rx;
  gpio_pins_t tx;
  gpio_pins_t inverter;
} target_serial_port_t;

#define TARGET_SERIAL_MEMBERS \
  MEMBER(index, uint8)        \
  MEMBER(rx, gpio_pins_t)     \
  MEMBER(tx, gpio_pins_t)     \
  MEMBER(inverter, gpio_pins_t)

typedef struct {
  uint8_t name[32];

  bool brushless;

  target_led_t leds[LED_MAX];
  target_serial_port_t serial_ports[SERIAL_PORT_MAX];
  target_serial_port_t serial_soft_ports[SERIAL_SOFT_COUNT];

  gpio_pins_t usb_detect;
  gpio_pins_t fpv;
  gpio_pins_t vbat;
  gpio_pins_t ibat;

  target_invert_pin_t sdcard_detect;
  target_invert_pin_t buzzer;
  gpio_pins_t motor_pins[MOTOR_PIN_MAX];
} target_t;

#define TARGET_MEMBERS                                                     \
  TSTR_MEMBER(name, 32)                                                    \
  MEMBER(brushless, bool)                                                  \
  ARRAY_MEMBER(leds, LED_MAX, target_led_t)                                \
  ARRAY_MEMBER(serial_ports, SERIAL_PORT_MAX, target_serial_port_t)        \
  ARRAY_MEMBER(serial_soft_ports, SERIAL_SOFT_COUNT, target_serial_port_t) \
  MEMBER(usb_detect, gpio_pins_t)                                          \
  MEMBER(fpv, gpio_pins_t)                                                 \
  MEMBER(vbat, gpio_pins_t)                                                \
  MEMBER(ibat, gpio_pins_t)                                                \
  MEMBER(sdcard_detect, target_invert_pin_t)                               \
  MEMBER(buzzer, target_invert_pin_t)                                      \
  ARRAY_MEMBER(motor_pins, MOTOR_PIN_MAX, gpio_pins_t)

extern target_t target;

bool target_serial_port_valid(const target_serial_port_t *port);

cbor_result_t cbor_encode_gpio_pins_t(cbor_value_t *enc, const gpio_pins_t *t);
cbor_result_t cbor_decode_gpio_pins_t(cbor_value_t *dec, gpio_pins_t *t);

cbor_result_t cbor_encode_target_t(cbor_value_t *enc, const target_t *t);
cbor_result_t cbor_decode_target_t(cbor_value_t *dec, target_t *t);