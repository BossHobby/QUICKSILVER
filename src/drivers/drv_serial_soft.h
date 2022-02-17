#pragma once

#include "drv_gpio.h"
#include "drv_time.h"
#include "project.h"

typedef struct {
  gpio_pins_t tx_pin;
  gpio_pins_t rx_pin;

  uint32_t baud;
  uint8_t stop_bits;

  uint32_t cycles_per_bit;
  uint32_t cycles_per_bit_half;
} soft_serial_t;

uint8_t soft_serial_init(soft_serial_t *dev, gpio_pins_t tx_pin, gpio_pins_t rx_pin, uint32_t baudrate, uint8_t stop_bits);

void soft_serial_set_input(const soft_serial_t *data);
void soft_serial_set_output(const soft_serial_t *data);

uint8_t soft_serial_read_byte(const soft_serial_t *dev, uint8_t *byte);
void soft_serial_write_byte(const soft_serial_t *dev, uint8_t byte);

uint8_t soft_serial_read_bytes(const soft_serial_t *dev, uint8_t *byte, uint32_t size);
void soft_serial_write_bytes(const soft_serial_t *dev, uint8_t *byte, uint32_t size);