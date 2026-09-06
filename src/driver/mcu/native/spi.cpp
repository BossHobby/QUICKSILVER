#include "driver/spi.h"

#include <string.h>

#include "core/failloop.h"
#include "driver/interrupt.h"

// Native SPI port definitions (simulated)
const spi_port_def_t spi_port_defs[SPI_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = (spi_port_t *)1,  // Simulated port 1
        .rcc = 0,
        .dma_rx = DMA_DEVICE_INVALID,
        .dma_tx = DMA_DEVICE_INVALID,
    },
    {
        .channel_index = 2,
        .channel = (spi_port_t *)2,  // Simulated port 2
        .rcc = 0,
        .dma_rx = DMA_DEVICE_INVALID,
        .dma_tx = DMA_DEVICE_INVALID,
    },
    {
        .channel_index = 3,
        .channel = (spi_port_t *)3,  // Simulated port 3
        .rcc = 0,
        .dma_rx = DMA_DEVICE_INVALID,
        .dma_tx = DMA_DEVICE_INVALID,
    },
    {
        .channel_index = 4,
        .channel = (spi_port_t *)4,  // Simulated port 4
        .rcc = 0,
        .dma_rx = DMA_DEVICE_INVALID,
        .dma_tx = DMA_DEVICE_INVALID,
    },
};

extern FAST_RAM spi_txn_t txn_pool[SPI_TXN_MAX];

void spi_device_init(spi_ports_t port) {
  if (port >= SPI_PORT_MAX || port == 0) {
    return;
  }

  spi_device_t *dev = &spi_dev[port];
  
  dev->is_init = true;
  dev->dma_done = true;
  dev->txn_head = 0;
  dev->txn_tail = 0;
  dev->mode = SPI_MODE_LEADING_EDGE;
  dev->hz = 1000000;  // Default 1MHz
}

void spi_reconfigure(spi_bus_device_t *bus) {
  if (bus->port >= SPI_PORT_MAX || bus->port == 0) {
    return;
  }
  
  spi_device_t *dev = &spi_dev[bus->port];
  if (!dev->is_init) {
    spi_device_init(bus->port);
  }
  
  dev->mode = bus->mode;
  dev->hz = bus->hz;
}

void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length, bool has_rx) {
  if (port >= SPI_PORT_MAX || port == 0) {
    return;
  }
  
  // Simulate SPI transfer - in a real simulator this could interface with
  // external hardware simulation or just echo data back
  
  // For now, we'll just copy TX data to RX data if applicable
  if (has_rx) {
    // Simulate some response data
    for (uint32_t i = 0; i < length; i++) {
      if (buffer[i] != 0) {
        // Echo back inverted data for simulation
        buffer[i] = ~buffer[i];
      }
    }
  }
  
  // Mark DMA as complete immediately (no actual DMA in simulator)
  spi_dev[port].dma_done = true;
}

void spi_seg_submit_wait_ex(spi_bus_device_t *bus, const spi_txn_segment_t *segs, const uint32_t count) {
}

// Called by interrupt handler to finish SPI transaction
extern void spi_txn_finish(spi_ports_t port);

// Since we don't have actual hardware interrupts in the simulator,
// we need to poll for completion
void spi_dma_complete_port(spi_ports_t port) {
  if (port >= SPI_PORT_MAX || port == 0) {
    return;
  }
  
  if (spi_dev[port].dma_done) {
    spi_txn_finish(port);
  }
}