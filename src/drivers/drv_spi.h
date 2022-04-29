#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drv_gpio.h"
#include "project.h"
#include "spi_ports.h"

typedef struct {
  DMA_TypeDef *dma;
  uint32_t dma_port;
  uint32_t channel;
  uint8_t channel_index;

  uint32_t rx_request;
  uint8_t rx_stream_index;
  DMA_Stream_TypeDef *rx_stream;
  IRQn_Type rx_it;

  uint32_t tx_request;
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

#define SPI_TXN_MAX 16
#define SPI_TXN_SEG_MAX 8

typedef struct {
  bool live;
  const uint8_t *tx_data;
  uint8_t *rx_data;
  uint32_t size;
} spi_txn_segment_t;

struct spi_bus_device;

typedef enum {
  SPI_MODE_INVALID,
  SPI_MODE_LEADING_EDGE,
  SPI_MODE_TRAILING_EDGE,
} spi_mode_t;

typedef enum {
  TXN_WAITING,
  TXN_READY,
  TXN_IN_PROGRESS,
  TXN_DONE,
  TXN_ERROR,
} spi_txn_status_t;

typedef void (*spi_txn_done_fn_t)();

typedef struct {
  volatile struct spi_bus_device *bus;

  spi_txn_status_t status;

  spi_txn_segment_t segments[SPI_TXN_SEG_MAX];
  uint8_t segment_count;

  uint32_t offset;
  uint32_t size;

  spi_txn_done_fn_t done_fn;
} spi_txn_t;

typedef struct spi_bus_device {
  spi_ports_t port;
  gpio_pins_t nss;

  volatile uint8_t *buffer;
  uint32_t buffer_size;

  bool auto_continue;
  bool (*poll_fn)();

  // only modified by the main loop
  uint8_t txn_head;
  // only modified by the intterupt or protected code
  uint8_t txn_tail;

  spi_txn_t txns[SPI_TXN_MAX];

  spi_mode_t mode;
  uint32_t divider;
} spi_bus_device_t;

extern const spi_port_def_t spi_port_defs[SPI_PORTS_MAX];

uint32_t spi_find_divder(uint32_t clk_hz);

uint8_t spi_dma_is_ready(spi_ports_t port);
bool spi_dma_wait_for_ready(spi_ports_t port);

void spi_bus_device_init(volatile spi_bus_device_t *bus);
void spi_bus_device_reconfigure(volatile spi_bus_device_t *bus, spi_mode_t mode, uint32_t divider);

spi_txn_t *spi_txn_init(volatile spi_bus_device_t *bus, spi_txn_done_fn_t done_fn);
void spi_txn_add_seg(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size);
void spi_txn_add_seg_delay(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size);
void spi_txn_add_seg_const(spi_txn_t *txn, const uint8_t tx_data);
void spi_txn_submit(spi_txn_t *txn);

void spi_txn_continue(volatile spi_bus_device_t *bus);
bool spi_txn_ready(volatile spi_bus_device_t *bus);
void spi_txn_wait(volatile spi_bus_device_t *bus);