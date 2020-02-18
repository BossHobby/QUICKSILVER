#pragma once

#include <stdint.h>

#include "drv_gpio.h"
#include "spi_ports.h"

#include "project.h"

typedef struct {
  uint8_t channel_index;
  SPI_TypeDef *channel;

  uint32_t gpio_af;

  gpio_pins_t sck;
  gpio_pins_t miso;
  gpio_pins_t mosi;
} spi_port_def_t;

extern spi_port_def_t spi_port_defs[SPI_PORTS_MAX];

void spi_enable_rcc(spi_ports_t port);
uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data);

// soft spi  header file
void spi_init(void);
void spi_cson(void);
void spi_csoff(void);
void spi_sendbyte(int);
int spi_sendrecvbyte(int);
int spi_sendzerorecvbyte(void);
