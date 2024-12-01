#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
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

typedef struct {
  uint8_t *rx_data;
  uint32_t size;
} spi_txn_segment_t;

struct spi_bus_device;

typedef enum {
  TXN_IDLE,
  TXN_WAITING,
  TXN_READY,
  TXN_IN_PROGRESS,
} spi_txn_status_t;

typedef enum {
  TXN_DELAYED_RX = (1 << 1),
} spi_txn_flags_t;

typedef void (*spi_txn_done_fn_t)(void *arg);

typedef struct {
  struct spi_bus_device *bus;

  volatile spi_txn_status_t status;

  uint8_t flags;
  spi_txn_segment_t segments[SPI_TXN_SEG_MAX];
  uint8_t segment_count;

  uint8_t *buffer;
  uint32_t size;

  spi_txn_done_fn_t done_fn;
  void *done_fn_arg;
} spi_txn_t;

typedef struct {
  spi_txn_done_fn_t done_fn;
  void *done_fn_arg;
} spi_txn_opts_t;

typedef struct spi_bus_device {
  spi_ports_t port;
  gpio_pins_t nss;

  bool (*poll_fn)();

  spi_mode_t mode;
  uint32_t hz;
} spi_bus_device_t;

typedef struct {
  bool is_init;
  volatile bool dma_done;

  // only modified by the main loop
  volatile uint8_t txn_head;
  // only modified by the intterupt or protected code
  volatile uint8_t txn_tail;

  spi_txn_t *txns[SPI_TXN_MAX];

  spi_mode_t mode;
  uint32_t hz;
} spi_device_t;

extern spi_device_t spi_dev[SPI_PORT_MAX];
extern const spi_port_def_t spi_port_defs[SPI_PORT_MAX];

void spi_bus_device_init(const spi_bus_device_t *bus);

void spi_txn_wait(spi_bus_device_t *bus);
spi_txn_t *spi_txn_pop(spi_bus_device_t *bus, const spi_txn_opts_t opt);
void spi_txn_push(spi_bus_device_t *bus, spi_txn_t *txn);
bool spi_txn_continue_port(spi_ports_t port);

void spi_seg_transfer_start(spi_bus_device_t *bus);
void spi_seg_transfer_bytes(spi_bus_device_t *bus, const uint8_t *tx_data, uint8_t *rx_data, const uint32_t size);
void spi_seg_transfer_finish(spi_bus_device_t *bus);

static inline void spi_csn_enable(spi_bus_device_t *bus) { gpio_pin_reset(bus->nss); }
static inline void spi_csn_disable(spi_bus_device_t *bus) { gpio_pin_set(bus->nss); }

static inline bool spi_dma_is_ready(spi_ports_t port) { return spi_dev[port].dma_done; }
static inline bool spi_txn_ready(spi_bus_device_t *bus) { return spi_dev[bus->port].txn_head == spi_dev[bus->port].txn_tail; }

static inline bool spi_txn_continue(spi_bus_device_t *bus) { return spi_txn_continue_port(bus->port); }
static inline void spi_txn_add_seg(spi_txn_t *txn, const uint8_t *tx_data, uint8_t *rx_data, const uint32_t size) {
  spi_txn_segment_t *seg = &txn->segments[txn->segment_count];
  seg->rx_data = rx_data;
  if (seg->rx_data)
    txn->flags |= TXN_DELAYED_RX;

  for (uint32_t i = 0; i < size; i++)
    txn->buffer[txn->size + i] = tx_data ? tx_data[i] : 0xFF;

  txn->size += (seg->size = size);
  txn->segment_count++;
}

static inline void spi_bus_device_reconfigure(spi_bus_device_t *bus, spi_mode_t mode, uint32_t hz) {
  bus->mode = mode;
  bus->hz = hz;
}

#define spi_make_seg_buffer(_rx_data, _tx_data, _size)            \
  {                                                               \
    if (__is_async__)                                             \
      spi_txn_add_seg(txn, _tx_data, _rx_data, _size);            \
    else                                                          \
      spi_seg_transfer_bytes(__bus__, _tx_data, _rx_data, _size); \
  }
#define spi_make_seg_const(_bytes...)               \
  {                                                 \
    const uint8_t __bytes__[] = {_bytes};           \
    const uint32_t __size__ = sizeof(__bytes__);    \
    spi_make_seg_buffer(NULL, __bytes__, __size__); \
  }
#define spi_seg_submit_wait(_bus, _segs) \
  {                                      \
    const bool __is_async__ = false;     \
    spi_bus_device_t *__bus__ = _bus;    \
    spi_txn_t *txn = NULL;               \
    spi_seg_transfer_start(__bus__);     \
    _segs;                               \
    spi_seg_transfer_finish(__bus__);    \
  }
#define spi_seg_submit(_bus, _segs, _opts...)        \
  {                                                  \
    const bool __is_async__ = true;                  \
    spi_bus_device_t *__bus__ = _bus;                \
    const spi_txn_opts_t __opts__ = {_opts};         \
    spi_txn_t *txn = spi_txn_pop(__bus__, __opts__); \
    _segs;                                           \
    spi_txn_push(__bus__, txn);                      \
  }

static inline void spi_txn_set_done(void *arg) { *((bool *)arg) = true; }
#define spi_seg_submit_check(_bus, _segs, ...)             \
  ({                                                       \
    static volatile bool __did_submit__ = false;           \
    static volatile bool __is_done__ = false;              \
    if (!__did_submit__) {                                 \
      __VA_ARGS__;                                         \
      spi_seg_submit(_bus, _segs,                          \
                     .done_fn = spi_txn_set_done,          \
                     .done_fn_arg = (void *)&__is_done__); \
      spi_txn_continue(_bus);                              \
      __did_submit__ = true;                               \
    }                                                      \
    const bool temp = __is_done__;                         \
    if (__is_done__) {                                     \
      __did_submit__ = false;                              \
      __is_done__ = false;                                 \
    }                                                      \
    temp;                                                  \
  })
