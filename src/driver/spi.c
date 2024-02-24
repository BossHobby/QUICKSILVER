#include "driver/spi.h"

#include <string.h>

#include "core/failloop.h"
#include "driver/interrupt.h"

FAST_RAM spi_port_config_t spi_port_config[SPI_PORT_MAX];
FAST_RAM volatile uint8_t dma_transfer_done[16] = {[0 ... 15] = 1};
FAST_RAM spi_txn_t txn_pool[SPI_TXN_MAX];

extern void spi_reconfigure(spi_bus_device_t *bus);

extern void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length);

void spi_enable_rcc(spi_ports_t port) {
  rcc_enable(spi_port_defs[port].rcc);
}

static inline __attribute__((always_inline)) spi_txn_t *spi_txn_pop() {
  ATOMIC_BLOCK_ALL {
    for (uint32_t i = 0; i < SPI_TXN_MAX; i++) {
      if (txn_pool[i].status == TXN_IDLE) {
        spi_txn_t *txn = &txn_pool[i];
        txn->status = TXN_WAITING;
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

  const spi_port_config_t *config = &spi_port_config[bus->port];
  if (config->active_device != NULL && config->active_device != bus) {
    return false;
  }

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

void spi_txn_continue(spi_bus_device_t *bus) {
  ATOMIC_BLOCK_ALL {
    if (bus->txn_head == bus->txn_tail) {
      return;
    }

    if (!spi_txn_can_send(bus, true)) {
      return;
    }

    const uint32_t tail = (bus->txn_tail + 1) % SPI_TXN_MAX;
    spi_txn_t *txn = bus->txns[tail];

    spi_port_config[bus->port].active_device = bus;
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

    spi_reconfigure(bus);

    spi_csn_enable(bus);
    spi_dma_transfer_begin(bus->port, txn->buffer, txn->size);
  }
}

void spi_seg_submit_ex(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn, const spi_txn_segment_t *segs, const uint32_t count) {
  spi_txn_t *txn = spi_txn_pop();
  if (txn == NULL) {
    failloop(FAILLOOP_SPI);
  }
  if (count >= SPI_TXN_SEG_MAX) {
    failloop(FAILLOOP_SPI);
  }

  txn->bus = bus;
  txn->segment_count = count;

  txn->flags = 0;
  txn->size = 0;
  txn->done_fn = done_fn;

  txn->buffer_size = 0;
  for (uint32_t i = 0; i < count; i++) {
    txn->buffer_size += segs[i].size;
  }
  txn->buffer = dma_mem_alloc(txn->buffer_size);

  for (uint32_t i = 0; i < count; i++) {
    const spi_txn_segment_t *seg = &segs[i];
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
    const uint8_t head = (txn->bus->txn_head + 1) % SPI_TXN_MAX;
    txn->status = TXN_READY;
    txn->bus->txns[head] = txn;
    txn->bus->txn_head = head;
  }
}

void spi_seg_submit_continue_ex(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn, const spi_txn_segment_t *segs, const uint32_t count) {
  spi_seg_submit_ex(bus, done_fn, segs, count);
  spi_txn_continue(bus);
}

void spi_txn_finish(spi_bus_device_t *bus) {
  const uint32_t tail = (bus->txn_tail + 1) % SPI_TXN_MAX;
  spi_txn_t *txn = bus->txns[tail];

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
    txn->done_fn();
  }

  dma_mem_free(txn->buffer);

  ATOMIC_BLOCK_ALL {
    txn->buffer = NULL;
    txn->status = TXN_IDLE;

    bus->txns[tail] = NULL;
    bus->txn_tail = tail;

    spi_port_config[bus->port].active_device = NULL;
  }
}