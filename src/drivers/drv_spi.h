#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drv_dma.h"
#include "drv_gpio.h"
#include "project.h"
#include "spi_ports.h"

typedef struct {
  uint8_t channel_index;
  SPI_TypeDef *channel;

  uint32_t gpio_af;

  gpio_pins_t sck;
  gpio_pins_t miso;
  gpio_pins_t mosi;

  dma_device_t dma_rx;
  dma_device_t dma_tx;
} spi_port_def_t;

#define SPI_TXN_MAX 16
#define SPI_TXN_SEG_MAX 8

typedef struct {
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
  struct spi_bus_device *bus;

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

  uint8_t *buffer;
  uint32_t buffer_size;

  bool auto_continue;
  bool (*poll_fn)();

  // only modified by the main loop
  volatile uint8_t txn_head;
  // only modified by the intterupt or protected code
  volatile uint8_t txn_tail;

  spi_txn_t txns[SPI_TXN_MAX];

  spi_mode_t mode;
  uint32_t hz;
} spi_bus_device_t;

extern const spi_port_def_t spi_port_defs[SPI_PORTS_MAX];

uint8_t spi_dma_is_ready(spi_ports_t port);

void spi_bus_device_init(spi_bus_device_t *bus);
void spi_bus_device_reconfigure(spi_bus_device_t *bus, spi_mode_t mode, uint32_t hz);

void spi_csn_enable(spi_bus_device_t *bus);
void spi_csn_disable(spi_bus_device_t *bus);

spi_txn_t *spi_txn_init(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn);
void spi_txn_add_seg(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size);
void spi_txn_add_seg_delay(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size);
void spi_txn_add_seg_const(spi_txn_t *txn, const uint8_t tx_data);
void spi_txn_submit(spi_txn_t *txn);

void spi_txn_continue(spi_bus_device_t *bus);
void spi_txn_continue_ex(spi_bus_device_t *bus, bool force_sync);
bool spi_txn_ready(spi_bus_device_t *bus);
void spi_txn_wait(spi_bus_device_t *bus);
void spi_txn_submit_wait(spi_bus_device_t *bus, spi_txn_t *txn);
void spi_txn_submit_continue(spi_bus_device_t *bus, spi_txn_t *txn);