#pragma once

#include "target.h"

#ifndef SPI_PORTS
#define SPI_PORTS
#endif

#ifdef F4
#define SPI3_PB3PB4PB5 SPI_PORT(3, PIN_B3, PIN_B4, PIN_B5)
#endif

#define SPI_IDENT(channel) SPI_PORT##channel
#define SPI_PORT(channel, sck_pin, miso_pin, mosi_pin) SPI_IDENT(channel),

typedef enum {
  SPI_PORT_INVALID,
  SPI_PORTS
  SPI_PORTS_MAX,
} spi_ports_t;

#undef SPI_PORT