#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/rcc.h"

typedef enum {
  SPI_MODE_INVALID,
  SPI_MODE_LEADING_EDGE,
  SPI_MODE_TRAILING_EDGE,
} spi_mode_t;

typedef struct {
  uint8_t channel_index;
  spi_port_t *channel;
  rcc_reg_t rcc;
  dma_device_t dma_rx;
  dma_device_t dma_tx;
} spi_port_def_t;

#define SPI_TXN_MAX 32
#define SPI_TXN_SEG_MAX 8

typedef enum {
  TXN_CONST,
  TXN_BUFFER,
  TXN_DELAY,
} spi_txn_segment_type_t;

typedef struct {
  spi_txn_segment_type_t type;
  union {
    struct {
      uint8_t byte;
    };
    struct {
      const uint8_t *tx_data;
      uint8_t *rx_data;
    };
  };
  uint32_t size;
} spi_txn_segment_t;

#define spi_make_seg_const(_byte) \
  (spi_txn_segment_t) { .type = TXN_CONST, .byte = (_byte), .size = 1 }
#define spi_make_seg_buffer(_rx_data, _tx_data, _size) \
  (spi_txn_segment_t) { .type = TXN_BUFFER, .rx_data = (_rx_data), .tx_data = (_tx_data), .size = (_size) }
#define spi_make_seg_delay(_rx_data, _tx_data, _size) \
  (spi_txn_segment_t) { .type = TXN_DELAY, .rx_data = (_rx_data), .tx_data = (_tx_data), .size = (_size) }

struct spi_bus_device;

typedef enum {
  TXN_IDLE,
  TXN_WAITING,
  TXN_READY,
  TXN_IN_PROGRESS,
} spi_txn_status_t;

typedef enum {
  TXN_DELAYED_TX = (1 << 0),
  TXN_DELAYED_RX = (1 << 1),
} spi_txn_flags_t;

typedef void (*spi_txn_done_fn_t)();

typedef struct {
  struct spi_bus_device *bus;

  volatile spi_txn_status_t status;

  uint8_t flags;
  spi_txn_segment_t segments[SPI_TXN_SEG_MAX];
  uint8_t segment_count;

  uint8_t *buffer;
  uint32_t buffer_size;

  uint32_t size;

  spi_txn_done_fn_t done_fn;
} spi_txn_t;

typedef struct spi_bus_device {
  spi_ports_t port;
  gpio_pins_t nss;

  bool auto_continue;
  bool (*poll_fn)();

  // only modified by the main loop
  volatile uint8_t txn_head;
  // only modified by the intterupt or protected code
  volatile uint8_t txn_tail;

  spi_txn_t *txns[SPI_TXN_MAX];

  spi_mode_t mode;
  uint32_t hz;
} spi_bus_device_t;

typedef struct {
  spi_bus_device_t *active_device;
  spi_mode_t mode;
  uint32_t hz;
} spi_port_config_t;

extern const spi_port_def_t spi_port_defs[SPI_PORT_MAX];

uint8_t spi_dma_is_ready(spi_ports_t port);

void spi_bus_device_init(spi_bus_device_t *bus);
void spi_bus_device_reconfigure(spi_bus_device_t *bus, spi_mode_t mode, uint32_t hz);

bool spi_txn_ready(spi_bus_device_t *bus);
void spi_txn_continue(spi_bus_device_t *bus);
void spi_txn_wait(spi_bus_device_t *bus);

void spi_seg_submit_ex(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn, const spi_txn_segment_t *segs, const uint32_t count);
void spi_seg_submit_wait_ex(spi_bus_device_t *bus, const spi_txn_segment_t *segs, const uint32_t count);
void spi_seg_submit_continue_ex(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn, const spi_txn_segment_t *segs, const uint32_t count);

#define spi_seg_submit(bus, done_fn, segs) spi_seg_submit_ex(bus, done_fn, segs, sizeof(segs) / sizeof(spi_txn_segment_t))
#define spi_seg_submit_wait(bus, segs) spi_seg_submit_wait_ex(bus, segs, sizeof(segs) / sizeof(spi_txn_segment_t))
#define spi_seg_submit_continue(bus, done_fn, segs) spi_seg_submit_continue_ex(bus, done_fn, segs, sizeof(segs) / sizeof(spi_txn_segment_t))