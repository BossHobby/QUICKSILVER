#pragma once

#include <stdbool.h>

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"
#include "driver/timer.h"

typedef struct {
  uint32_t baud;
  uint8_t stop_bits;
  resource_tag_t timer;

  bool tx_active;
  uint8_t tx_byte;
  uint8_t tx_state;

  bool rx_active;
  uint8_t rx_byte;
  uint8_t rx_state;
} soft_serial_t;

uint8_t soft_serial_init(serial_ports_t port, uint32_t baudrate, uint8_t stop_bits);

void soft_serial_enable_write(serial_ports_t port);
void soft_serial_enable_read(serial_ports_t port);

uint8_t soft_serial_read_byte(serial_ports_t port);
void soft_serial_write_byte(serial_ports_t port, uint8_t byte);