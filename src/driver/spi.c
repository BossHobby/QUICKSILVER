#include "driver/spi.h"

#include <string.h>

#include "core/failloop.h"
#include "driver/interrupt.h"

FAST_RAM spi_device_t spi_dev[SPI_PORT_MAX] = {
    [RANGE_INIT(0, SPI_PORT_MAX)] = {.is_init = false, .dma_done = true, .txn_head = 0, .txn_tail = 0},
};
FAST_RAM spi_txn_t txn_pool[SPI_TXN_MAX];
DMA_RAM uint8_t txn_buffers[SPI_TXN_MAX][DMA_ALIGN(512)];

extern void spi_reconfigure(spi_bus_device_t *bus);
extern void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length);

void spi_enable_rcc(spi_ports_t port) {
  rcc_enable(spi_port_defs[port].rcc);
}

static inline __attribute__((always_inline)) spi_txn_t *spi_txn_pop(spi_bus_device_t *bus) {
  ATOMIC_BLOCK_ALL {
    for (uint32_t i = 0; i < SPI_TXN_MAX; i++) {
      if (txn_pool[i].status == TXN_IDLE) {
        spi_txn_t *txn = &txn_pool[i];
        txn->status = TXN_WAITING;
        txn->buffer = txn_buffers[i];
        return txn;
      }
    }
  }
  return NULL;
}

bool spi_txn_can_send(spi_bus_device_t *bus, bool dma) {
  if (!spi_dma_is_ready(bus->port)) {
    return false;
  }

#if defined(STM32F4) && defined(USE_MOTOR_DSHOT)
  extern volatile uint32_t dshot_dma_phase;
  if (dma &&
      target.brushless &&
      bus->port == SPI_PORT1 &&
      dshot_dma_phase != 0) {
    return false;
  }
#endif

  if (bus->poll_fn && !bus->poll_fn()) {
    return false;
  }

  return true;
}

void spi_txn_wait(spi_bus_device_t *bus) {
  while (!spi_txn_ready(bus)) {
    spi_txn_continue(bus);
  }
}

bool spi_txn_continue_port(spi_ports_t port) {
  spi_device_t *dev = &spi_dev[port];
  spi_txn_t *txn = NULL;

  ATOMIC_BLOCK_ALL {
    if (dev->txn_head == dev->txn_tail) {
      return false;
    }

    txn = dev->txns[(dev->txn_tail + 1) % SPI_TXN_MAX];
    if (txn->status != TXN_READY) {
      return false;
    }

    if (!spi_txn_can_send(txn->bus, true)) {
      return false;
    }

    txn->status = TXN_IN_PROGRESS;

    if (txn->flags & TXN_DELAYED_TX) {
      uint32_t txn_size = 0;
      for (uint32_t i = 0; i < txn->segment_count; ++i) {
        spi_txn_segment_t *seg = &txn->segments[i];
        if (seg->tx_data) {
          memcpy((uint8_t *)txn->buffer + txn_size, seg->tx_data, seg->size);
        }
        txn_size += seg->size;
      }
    }

    dev->dma_done = false;
  }

  spi_reconfigure(txn->bus);
  spi_csn_enable(txn->bus);

  spi_dma_transfer_begin(port, txn->buffer, txn->size);

  return true;
}

void spi_seg_submit_ex(spi_bus_device_t *bus, const spi_txn_opts_t opts) {
  spi_txn_t *txn = spi_txn_pop(bus);
  if (txn == NULL) {
    failloop(FAILLOOP_SPI);
  }

  txn->bus = bus;
  txn->size = 0;
  txn->flags = 0;
  txn->segment_count = opts.seg_count;
  txn->done_fn = opts.done_fn;
  txn->done_fn_arg = opts.done_fn_arg;

  for (uint32_t i = 0; i < opts.seg_count; i++) {
    const spi_txn_segment_t *seg = &opts.segs[i];
    spi_txn_segment_t *txn_seg = &txn->segments[i];

    switch (seg->type) {
    case TXN_CONST:
      txn_seg->rx_data = NULL;
      txn_seg->tx_data = NULL;
      memcpy(txn->buffer + txn->size, seg->bytes, seg->size);
      break;

    case TXN_BUFFER:
      txn_seg->tx_data = NULL;
      txn_seg->rx_data = seg->rx_data;
      if (seg->tx_data) {
        memcpy(txn->buffer + txn->size, seg->tx_data, seg->size);
      } else {
        memset(txn->buffer + txn->size, 0xFF, seg->size);
      }
      break;

    case TXN_DELAY:
      txn_seg->tx_data = seg->tx_data;
      txn_seg->rx_data = seg->rx_data;
      txn->flags |= TXN_DELAYED_TX;
      break;
    }

    if (txn_seg->rx_data) {
      txn->flags |= TXN_DELAYED_RX;
    }

    txn_seg->size = seg->size;
    txn->size += seg->size;
  }

  ATOMIC_BLOCK_ALL {
    spi_device_t *dev = &spi_dev[bus->port];
    const uint8_t head = (dev->txn_head + 1) % SPI_TXN_MAX;
    txn->status = TXN_READY;
    dev->txns[head] = txn;
    dev->txn_head = head;
  }
}

// only called from dma isr
void spi_txn_finish(spi_ports_t port) {
  spi_device_t *dev = &spi_dev[port];

  const uint32_t tail = (dev->txn_tail + 1) % SPI_TXN_MAX;
  spi_txn_t *txn = dev->txns[tail];

  spi_csn_disable(txn->bus);

  if (txn->flags & TXN_DELAYED_RX) {
    uint32_t txn_size = 0;
    for (uint32_t i = 0; i < txn->segment_count; ++i) {
      spi_txn_segment_t *seg = &txn->segments[i];
      if (seg->rx_data) {
        memcpy(seg->rx_data, (uint8_t *)txn->buffer + txn_size, seg->size);
      }
      txn_size += seg->size;
    }
  }

  if (txn->done_fn) {
    txn->done_fn(txn->done_fn_arg);
  }

  txn->status = TXN_IDLE;

  dev->txns[tail] = NULL;
  dev->txn_tail = tail;

  spi_dev[port].dma_done = true;
  spi_txn_continue_port(port);
}