#pragma once

#include <stdint.h>

#include "drv_gpio.h"
#include "spi_ports.h"

#include "project.h"

typedef struct {
  DMA_TypeDef *dma;
  uint32_t dma_port;
  uint32_t channel;
  uint8_t channel_index;

  uint8_t rx_stream_index;
  DMA_Stream_TypeDef *rx_stream;
  IRQn_Type rx_it;

  uint8_t tx_stream_index;
  DMA_Stream_TypeDef *tx_stream;
  IRQn_Type tx_it;
} spi_dma_def_t;

typedef struct {
  uint8_t channel_index;
  SPI_TypeDef *channel;

  uint32_t gpio_af;

  gpio_pins_t sck;
  gpio_pins_t miso;
  gpio_pins_t mosi;

  spi_dma_def_t dma;
} spi_port_def_t;

extern const volatile spi_port_def_t spi_port_defs[SPI_PORTS_MAX];

void spi_enable_rcc(spi_ports_t port);
void spi_init_pins(spi_ports_t port, gpio_pins_t nss);

void spi_csn_enable(gpio_pins_t nss);
void spi_csn_disable(gpio_pins_t nss);

uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data);
uint8_t spi_transfer_byte_timeout(spi_ports_t port, uint8_t data, uint32_t timeout);

void spi_dma_init(spi_ports_t port);
void spi_dma_enable_rcc(spi_ports_t port);

void spi_dma_receive_init(spi_ports_t port, uint8_t *base_address_in, uint32_t buffer_size);
void spi_dma_transmit_init(spi_ports_t port, uint8_t *base_address_out, uint32_t buffer_size);

uint8_t spi_dma_is_ready(spi_ports_t port);
void spi_dma_wait_for_ready(spi_ports_t port);
void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length);
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length);

// soft spi  header file
void spi_init(void);
void spi_cson(void);
void spi_csoff(void);
void spi_sendbyte(int);
int spi_sendrecvbyte(int);
int spi_sendzerorecvbyte(void);
