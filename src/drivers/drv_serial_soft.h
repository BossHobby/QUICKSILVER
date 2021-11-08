#pragma once

#include "drv_gpio.h"
#include "drv_time.h"
#include "project.h"

typedef struct {
  gpio_pins_t tx_pin;
  gpio_pins_t rx_pin;

  uint32_t baud;

  uint32_t micros_per_bit;
  uint32_t micros_per_bit_half;
} soft_serial_t;

uint8_t soft_serial_init(soft_serial_t *dev, gpio_pins_t tx_pin, gpio_pins_t rx_pin, uint32_t baudrate);

void soft_serial_set_input(const soft_serial_t *data);
void soft_serial_set_output(const soft_serial_t *data);

uint8_t soft_serial_read_byte(const soft_serial_t *dev, uint8_t *byte);
void soft_serial_write_byte(const soft_serial_t *dev, uint8_t byte);
