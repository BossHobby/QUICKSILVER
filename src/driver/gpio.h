#pragma once

#include "core/project.h"
#include "driver/resource.h"
#include "driver/timer.h"

typedef enum {
  GPIO_OUTPUT,
  GPIO_INPUT,
  GPIO_ANALOG,
  GPIO_ALTERNATE,
} gpio_mode_t;

typedef enum {
  GPIO_PUSHPULL,
  GPIO_OPENDRAIN,
} gpio_output_t;

typedef enum {
  GPIO_DRIVE_NORMAL,
  GPIO_DRIVE_HIGH,
} gpio_drive_t;

typedef enum {
  GPIO_NO_PULL,
  GPIO_UP_PULL,
  GPIO_DOWN_PULL,
} gpio_pull_t;

typedef struct {
  gpio_mode_t mode;
  gpio_output_t output;
  gpio_drive_t drive;
  gpio_pull_t pull;
} gpio_config_t;

typedef struct {
  gpio_pins_t pin;
  resource_tag_t tag;
  uint8_t af;
} gpio_af_t;

typedef struct {
  gpio_port_t *port;
  uint8_t pin_index;
  uint32_t pin;
} gpio_pin_def_t;

extern const gpio_pin_def_t gpio_pin_defs[PINS_MAX];
extern const gpio_af_t gpio_pin_afs[];
extern const uint32_t GPIO_AF_MAX;

#define gpio_config_default() { \
    .mode = GPIO_OUTPUT,        \
    .output = GPIO_PUSHPULL,    \
    .drive = GPIO_DRIVE_NORMAL, \
    .pull = GPIO_NO_PULL,       \
}

void gpio_ports_init();

void gpio_pin_init(gpio_pins_t pin, gpio_config_t config);
void gpio_pin_init_af(gpio_pins_t pin, gpio_config_t config, uint8_t af);
void gpio_pin_init_tag(gpio_pins_t pin, gpio_config_t config, resource_tag_t tag);

bool gpio_init_fpv(uint8_t mode);
