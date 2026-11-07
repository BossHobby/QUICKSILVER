#include "driver/spi.h"

#include <hardware/spi.h>

#include "core/failloop.h"
#include "driver/interrupt.h"

#ifdef USE_SPI

extern bool spi_txn_can_send(spi_bus_device_t *bus, bool dma);
extern void spi_txn_finish(spi_ports_t port);

const spi_port_def_t spi_port_defs[SPI_PORT_MAX] = {
    {},
    {
        .channel_index = 0,
        .channel = spi0_hw,
        .rcc = RESETS_RESET_SPI0_BITS,
        .dma_rx = DMA_DEVICE_SPI1_RX,
        .dma_tx = DMA_DEVICE_SPI1_TX,
    },
    {
        .channel_index = 1,
        .channel = spi1_hw,
        .rcc = RESETS_RESET_SPI1_BITS,
        .dma_rx = DMA_DEVICE_SPI2_RX,
        .dma_tx = DMA_DEVICE_SPI2_TX,
    },
};

// Channels used for each port's transfers, taken from the target or claimed.
static dma_stream_t spi_dma_rx[SPI_PORT_MAX];
static dma_stream_t spi_dma_tx[SPI_PORT_MAX];

static spi_inst_t *spi_inst(spi_ports_t port) {
  return (spi_inst_t *)spi_port_defs[port].channel;
}

static dma_stream_t spi_dma_stream(dma_device_t device) {
  const dma_stream_t stream = target.dma[device].dma;
  if (stream != DMA_STREAM_INVALID)
    return stream;
  return dma_claim_unused(device);
}

static void spi_dma_init(spi_ports_t port) {
  spi_inst_t *spi = spi_inst(port);
  const spi_port_def_t *def = &spi_port_defs[port];

  spi_dma_rx[port] = spi_dma_stream(def->dma_rx);
  spi_dma_tx[port] = spi_dma_stream(def->dma_tx);
  if (spi_dma_rx[port] == DMA_STREAM_INVALID || spi_dma_tx[port] == DMA_STREAM_INVALID)
    failloop(FAILLOOP_DMA);

  const uint32_t rx = dma_stream_channel(spi_dma_rx[port]);
  dma_channel_config config = dma_channel_get_default_config(rx);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
  channel_config_set_dreq(&config, spi_get_dreq(spi, false));
  channel_config_set_read_increment(&config, false);
  channel_config_set_write_increment(&config, true);
  dma_channel_configure(rx, &config, nullptr, &spi_get_hw(spi)->dr, 0, false);
  dma_channel_set_irq0_enabled(rx, true);

  const uint32_t tx = dma_stream_channel(spi_dma_tx[port]);
  config = dma_channel_get_default_config(tx);
  channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
  channel_config_set_dreq(&config, spi_get_dreq(spi, true));
  channel_config_set_read_increment(&config, true);
  channel_config_set_write_increment(&config, false);
  dma_channel_configure(tx, &config, &spi_get_hw(spi)->dr, nullptr, 0, false);

  interrupt_enable(DMA_IRQ_0_IRQn, DMA_PRIORITY);
}

void spi_reconfigure(spi_bus_device_t *bus) {
  spi_device_t *config = &spi_dev[bus->port];
  spi_inst_t *spi = spi_inst(bus->port);

  if (config->hz != bus->hz) {
    config->hz = bus->hz;
    spi_set_baudrate(spi, bus->hz);
  }
  if (config->mode != bus->mode) {
    config->mode = bus->mode;
    if (bus->mode == SPI_MODE_LEADING_EDGE)
      spi_set_format(spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    else
      spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  }
}

// RX completes after TX; its interrupt ends the transaction.
void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length, bool has_rx) {
  const uint32_t rx = dma_stream_channel(spi_dma_rx[port]);
  const uint32_t tx = dma_stream_channel(spi_dma_tx[port]);

  dma_channel_set_write_addr(rx, buffer, false);
  dma_channel_set_trans_count(rx, length, false);
  dma_channel_set_read_addr(tx, buffer, false);
  dma_channel_set_trans_count(tx, length, false);
  dma_start_channel_mask((1u << rx) | (1u << tx));
}

void spi_device_init(spi_ports_t port) {
  // spi_init() also cycles the block through reset.
  spi_inst_t *spi = spi_inst(port);
  spi_init(spi, 1000000);
  spi_set_format(spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

  spi_dev[port].dma_done = true;
  spi_dev[port].mode = SPI_MODE_TRAILING_EDGE;
  spi_dev[port].hz = 0;

  spi_dma_init(port);
}

void spi_seg_submit_wait_ex(spi_bus_device_t *bus, const spi_txn_segment_t *segs, const uint32_t count) {
  spi_txn_wait(bus);

  while (!spi_txn_can_send(bus, false))
    ;

  const spi_ports_t port = bus->port;
  spi_inst_t *spi = spi_inst(port);

  spi_dev[port].dma_done = false;

  spi_reconfigure(bus);
  spi_csn_enable(bus);

  for (uint32_t i = 0; i < count; i++) {
    const spi_txn_segment_t *seg = &segs[i];

    const uint8_t *tx_data = NULL;
    uint8_t *rx_data = NULL;
    if (seg->type == TXN_CONST) {
      tx_data = seg->bytes;
    } else {
      tx_data = seg->tx_data;
      rx_data = seg->rx_data;
    }

    if (tx_data && rx_data)
      spi_write_read_blocking(spi, tx_data, rx_data, seg->size);
    else if (tx_data)
      spi_write_blocking(spi, tx_data, seg->size);
    else if (rx_data)
      spi_read_blocking(spi, 0xFF, rx_data, seg->size);
  }

  spi_csn_disable(bus);

  spi_dev[port].dma_done = true;
}

void spi_dma_isr(const dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
    spi_txn_finish(SPI_PORT1);
    break;
  case DMA_DEVICE_SPI2_RX:
    spi_txn_finish(SPI_PORT2);
    break;
  default:
    break;
  }
}

#endif
