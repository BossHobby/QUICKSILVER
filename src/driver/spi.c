#include "driver/spi.h"

#include <string.h>

#include "core/failloop.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"
#include "driver/time.h"
#include "io/usb_configurator.h"

typedef struct {
  spi_bus_device_t *active_device;
  spi_mode_t mode;
  uint32_t hz;
} spi_port_config_t;

const spi_port_def_t spi_port_defs[SPI_PORT_MAX] = {
    {},
    {
        .channel_index = 1,
        .channel = SPI1,
        .rcc = RCC_APB2_GRP1(SPI1),
        .dma_rx = DMA_DEVICE_SPI1_RX,
        .dma_tx = DMA_DEVICE_SPI1_TX,
    },
    {
        .channel_index = 2,
        .channel = SPI2,
        .rcc = RCC_APB1_GRP1(SPI2),
        .dma_rx = DMA_DEVICE_SPI2_RX,
        .dma_tx = DMA_DEVICE_SPI2_TX,
    },
    {
        .channel_index = 3,
        .channel = SPI3,
        .rcc = RCC_APB1_GRP1(SPI3),
        .dma_rx = DMA_DEVICE_SPI3_RX,
        .dma_tx = DMA_DEVICE_SPI3_TX,
    },
#if defined(STM32F7) || defined(STM32H7)
    {
        .channel_index = 4,
        .channel = SPI4,
        .rcc = RCC_APB2_GRP1(SPI4),
        .dma_rx = DMA_DEVICE_SPI4_RX,
        .dma_tx = DMA_DEVICE_SPI4_TX,
    },
#endif
};

FAST_RAM static volatile spi_port_config_t spi_port_config[SPI_PORT_MAX];
FAST_RAM static volatile uint8_t dma_transfer_done[16] = {[0 ... 15] = 1};

#define PORT spi_port_defs[port]

static void spi_enable_rcc(spi_ports_t port) {
  rcc_enable(PORT.rcc);
}

static void spi_csn_enable(spi_bus_device_t *bus) {
  gpio_pin_reset(bus->nss);
}

static void spi_csn_disable(spi_bus_device_t *bus) {
  gpio_pin_set(bus->nss);
}

static uint32_t spi_divider_to_ll(uint32_t divider) {
  switch (divider) {
  default:
  case 2:
    return LL_SPI_BAUDRATEPRESCALER_DIV2;
  case 4:
    return LL_SPI_BAUDRATEPRESCALER_DIV4;
  case 8:
    return LL_SPI_BAUDRATEPRESCALER_DIV8;
  case 16:
    return LL_SPI_BAUDRATEPRESCALER_DIV16;
  case 32:
    return LL_SPI_BAUDRATEPRESCALER_DIV32;
  case 64:
    return LL_SPI_BAUDRATEPRESCALER_DIV64;
  case 128:
    return LL_SPI_BAUDRATEPRESCALER_DIV128;
  case 256:
    return LL_SPI_BAUDRATEPRESCALER_DIV256;
  }
}

static uint32_t spi_find_divder(uint32_t clk_hz) {
  uint32_t divider = 2;
  uint32_t clock = SPI_CLOCK_FREQ_HZ / divider;

  while ((clock > clk_hz) && (divider < 256)) {
    divider = divider << 1;
    clock = clock >> 1;
  }

  return spi_divider_to_ll(divider);
}

static void spi_init_pins(spi_ports_t port, gpio_pins_t nss) {
  const target_spi_port_t *dev = &target.spi_ports[port];

  LL_GPIO_InitTypeDef gpio_init;

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_pin_init_tag(&gpio_init, dev->sck, SPI_TAG(port, RES_SPI_SCK));

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_tag(&gpio_init, dev->miso, SPI_TAG(port, RES_SPI_MISO));

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_tag(&gpio_init, dev->mosi, SPI_TAG(port, RES_SPI_MOSI));

  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_pin_init(&gpio_init, nss);
  gpio_pin_set(nss);
}

static void spi_dma_init_rx(spi_ports_t port) {
  const dma_stream_def_t *dma = &dma_stream_defs[PORT.dma_rx];

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = dma->request;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&PORT.channel->RXDR;
#else
  DMA_InitStructure.Channel = dma->channel;
  DMA_InitStructure.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(PORT.channel);
#endif
  DMA_InitStructure.MemoryOrM2MDstAddress = 0;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.NbData = 0;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(dma->port, dma->stream_index, &DMA_InitStructure);
}

static void spi_dma_reset_rx(spi_ports_t port, uint8_t *rx_data, uint32_t rx_size) {
  const dma_stream_def_t *dma = &dma_stream_defs[PORT.dma_rx];

  while (LL_DMA_IsEnabledStream(dma->port, dma->stream_index))
    ;

  dma_prepare_rx_memory(rx_data, rx_size);

#ifdef STM32H7
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&PORT.channel->RXDR);
#else
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, LL_SPI_DMA_GetRegAddr(PORT.channel));
#endif
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)rx_data);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, rx_size);
}

static void spi_dma_init_tx(spi_ports_t port) {
  const dma_stream_def_t *dma = &dma_stream_defs[PORT.dma_tx];

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = dma->request;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&PORT.channel->TXDR;
#else
  DMA_InitStructure.Channel = dma->channel;
  DMA_InitStructure.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(PORT.channel);
#endif
  DMA_InitStructure.MemoryOrM2MDstAddress = 0;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.NbData = 0;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(dma->port, dma->stream_index, &DMA_InitStructure);
}

static void spi_dma_reset_tx(spi_ports_t port, uint8_t *tx_data, uint32_t tx_size) {
  const dma_stream_def_t *dma = &dma_stream_defs[PORT.dma_tx];

  while (LL_DMA_IsEnabledStream(dma->port, dma->stream_index))
    ;

  dma_prepare_tx_memory(tx_data, tx_size);

#ifdef STM32H7
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&PORT.channel->TXDR);
#else
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, LL_SPI_DMA_GetRegAddr(PORT.channel));
#endif
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)tx_data);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, tx_size);
}

static void spi_reconfigure(spi_bus_device_t *bus) {
  volatile spi_port_config_t *config = &spi_port_config[bus->port];
  const spi_port_def_t *port = &spi_port_defs[bus->port];
  const target_spi_port_t *dev = &target.spi_ports[bus->port];

  if (config->hz != bus->hz) {
    config->hz = bus->hz;
    LL_SPI_SetBaudRatePrescaler(port->channel, spi_find_divder(bus->hz));
  }
  if (config->mode != bus->mode) {
    config->mode = bus->mode;

    LL_GPIO_InitTypeDef gpio_init;
    gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
    gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;

    if (bus->mode == SPI_MODE_LEADING_EDGE) {
      gpio_init.Pull = LL_GPIO_PULL_DOWN;
      gpio_pin_init_tag(&gpio_init, dev->sck, SPI_TAG(bus->port, RES_SPI_SCK));

      LL_SPI_SetClockPhase(port->channel, LL_SPI_PHASE_1EDGE);
      LL_SPI_SetClockPolarity(port->channel, LL_SPI_POLARITY_LOW);
    } else if (bus->mode == SPI_MODE_TRAILING_EDGE) {
      gpio_init.Pull = LL_GPIO_PULL_UP;
      gpio_pin_init_tag(&gpio_init, dev->sck, SPI_TAG(bus->port, RES_SPI_SCK));

      LL_SPI_SetClockPhase(port->channel, LL_SPI_PHASE_2EDGE);
      LL_SPI_SetClockPolarity(port->channel, LL_SPI_POLARITY_HIGH);
    }
  }
}

static void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  dma_transfer_done[port] = 0;

#if !defined(STM32H7)
  // dummy read
  while (LL_SPI_IsActiveFlag_RXNE(PORT.channel))
    LL_SPI_ReceiveData8(PORT.channel);
#endif

  const dma_stream_def_t *dma_tx = &dma_stream_defs[PORT.dma_tx];
  const dma_stream_def_t *dma_rx = &dma_stream_defs[PORT.dma_rx];

  dma_clear_flag_tc(dma_rx->port, dma_rx->stream_index);
  dma_clear_flag_tc(dma_tx->port, dma_tx->stream_index);

  spi_dma_reset_rx(port, buffer, length);
  spi_dma_reset_tx(port, buffer, length);

  LL_DMA_EnableIT_TC(dma_rx->port, dma_rx->stream_index);
  LL_DMA_EnableIT_TE(dma_rx->port, dma_rx->stream_index);

  LL_DMA_EnableStream(dma_rx->port, dma_rx->stream_index);
  LL_DMA_EnableStream(dma_tx->port, dma_tx->stream_index);

  LL_SPI_EnableDMAReq_TX(PORT.channel);
  LL_SPI_EnableDMAReq_RX(PORT.channel);

#ifdef STM32H7
  LL_SPI_SetTransferSize(PORT.channel, length);
#endif
  LL_SPI_Enable(PORT.channel);
#ifdef STM32H7
  LL_SPI_StartMasterTransfer(PORT.channel);
#endif
}

uint8_t spi_dma_is_ready(spi_ports_t port) {
#if defined(STM32F4)
  if (target.brushless && port == SPI_PORT1) {
    extern volatile int dshot_dma_phase;
    if (dshot_dma_phase != 0) {
      return 0;
    }
  }
#endif
  return dma_transfer_done[port];
}

void spi_bus_device_init(spi_bus_device_t *bus) {
  if (!target_spi_port_valid(&target.spi_ports[bus->port])) {
    return;
  }

  bus->txn_head = 0;
  bus->txn_tail = 0;

  spi_init_pins(bus->port, bus->nss);
  spi_enable_rcc(bus->port);

  const spi_port_def_t *port = &spi_port_defs[bus->port];
  dma_enable_rcc(port->dma_rx);
  dma_enable_rcc(port->dma_tx);

  LL_SPI_DeInit(port->channel);

  LL_SPI_InitTypeDef default_init;
  default_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  default_init.Mode = LL_SPI_MODE_MASTER;
  default_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  default_init.ClockPolarity = LL_SPI_POLARITY_HIGH;
  default_init.ClockPhase = LL_SPI_PHASE_2EDGE;
  default_init.NSS = LL_SPI_NSS_SOFT;
  default_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  default_init.BitOrder = LL_SPI_MSB_FIRST;
  default_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  default_init.CRCPoly = 7;

#if defined(STM32H7)
  LL_SPI_EnableGPIOControl(port->channel);
  LL_SPI_SetFIFOThreshold(port->channel, LL_SPI_FIFO_TH_01DATA);
#endif
  LL_SPI_Init(port->channel, &default_init);

  spi_port_config[bus->port].mode = SPI_MODE_TRAILING_EDGE;
  spi_port_config[bus->port].hz = 0;

  spi_dma_init_rx(bus->port);
  spi_dma_init_tx(bus->port);

  const dma_stream_def_t *dma_rx = &dma_stream_defs[port->dma_rx];
  interrupt_enable(dma_rx->irq, DMA_PRIORITY);
  dma_transfer_done[bus->port] = 1;
}

void spi_bus_device_reconfigure(spi_bus_device_t *bus, spi_mode_t mode, uint32_t hz) {
  bus->mode = mode;
  bus->hz = hz;
}

static spi_txn_t *spi_txn_pop(spi_bus_device_t *bus) {
  ATOMIC_BLOCK_ALL {
    for (uint32_t i = 0; i < SPI_TXN_MAX; i++) {
      if (bus->txn_pool[i].status == TXN_IDLE) {
        spi_txn_t *txn = &bus->txn_pool[i];
        txn->status = TXN_WAITING;
        return txn;
      }
    }
  }
  return NULL;
}

bool spi_txn_ready(spi_bus_device_t *bus) {
  return bus->txn_head == bus->txn_tail;
}

static inline bool spi_txn_can_send(spi_bus_device_t *bus) {
  if (!spi_dma_is_ready(bus->port)) {
    return false;
  }

  volatile spi_port_config_t *config = &spi_port_config[bus->port];
  if (config->active_device != NULL && config->active_device != bus) {
    return false;
  }

  if (bus->poll_fn && !bus->poll_fn()) {
    return false;
  }

  return true;
}

void spi_txn_continue(spi_bus_device_t *bus) {
  ATOMIC_BLOCK_ALL {
    if (bus->txn_head == bus->txn_tail) {
      return;
    }

    if (!spi_txn_can_send(bus)) {
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
  spi_txn_t *txn = spi_txn_pop(bus);
  if (txn == NULL) {
    failloop(FAILLOOP_SPI);
  }

  txn->bus = bus;
  txn->segment_count = 0;

  txn->flags = 0;
  txn->size = 0;
  txn->done_fn = done_fn;

  txn->buffer_size = 0;
  for (uint32_t i = 0; i < count; i++) {
    txn->buffer_size += segs[i].size;
  }
  txn->buffer = dma_alloc(txn->buffer_size);

  spi_txn_segment_t *last_seg = NULL;
  for (uint32_t i = 0; i < count; i++) {
    const spi_txn_segment_t *seg = &segs[i];

    const uint8_t *tx_data = NULL;
    uint8_t *rx_data = NULL;

    switch (seg->type) {
    case TXN_CONST:
      txn->buffer[txn->size] = seg->byte;
      break;

    case TXN_BUFFER:
      if (seg->tx_data) {
        memcpy(txn->buffer + txn->size, seg->tx_data, seg->size);
      } else {
        memset(txn->buffer + txn->size, 0xFF, seg->size);
      }
      rx_data = seg->rx_data;
      break;

    case TXN_DELAY:
      tx_data = seg->tx_data;
      rx_data = seg->rx_data;
      txn->flags |= TXN_DELAYED_TX;
      break;
    }

    txn->size += seg->size;

    if (last_seg != NULL && last_seg->rx_data == rx_data && last_seg->tx_data == tx_data) {
      // merge segments
      last_seg->size += seg->size;
      continue;
    }

    if (txn->segment_count >= SPI_TXN_SEG_MAX) {
      failloop(FAILLOOP_SPI);
    }

    if (rx_data) {
      txn->flags |= TXN_DELAYED_RX;
    }

    last_seg = &txn->segments[txn->segment_count];
    last_seg->rx_data = rx_data;
    last_seg->tx_data = tx_data;
    last_seg->size = seg->size;
    txn->segment_count++;
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

void spi_seg_submit_wait_ex(spi_bus_device_t *bus, const spi_txn_segment_t *segs, const uint32_t count) {
  spi_txn_wait(bus);

  while (!spi_txn_can_send(bus))
    ;

  const spi_ports_t port = bus->port;

  spi_port_config[port].active_device = bus;
  dma_transfer_done[port] = 0;

  spi_reconfigure(bus);

  spi_csn_enable(bus);
#if defined(STM32H7)
  uint32_t transfer_size = 0;
  for (uint32_t i = 0; i < count; i++) {
    transfer_size += segs[i].size;
  }
  LL_SPI_SetTransferSize(PORT.channel, transfer_size);
  LL_SPI_Enable(PORT.channel);
  LL_SPI_StartMasterTransfer(PORT.channel);
#else
  LL_SPI_Enable(PORT.channel);
#endif

  for (uint32_t i = 0; i < count; i++) {
    const spi_txn_segment_t *seg = &segs[i];
    const uint32_t size = seg->size;

    const uint8_t *tx_data = NULL;
    uint8_t *rx_data = NULL;

    if (seg->type == TXN_CONST) {
      tx_data = &seg->byte;
    } else {
      tx_data = seg->tx_data;
      rx_data = seg->rx_data;
    }

    for (uint32_t j = 0; j < size; j++) {
#if defined(STM32H7)
      while (!LL_SPI_IsActiveFlag_TXP(PORT.channel))
        ;
#else
      while (!LL_SPI_IsActiveFlag_TXE(PORT.channel))
        ;
#endif
      LL_SPI_TransmitData8(PORT.channel, tx_data ? tx_data[j] : 0xFF);
#if defined(STM32H7)
      while (!LL_SPI_IsActiveFlag_RXP(PORT.channel))
        ;
#else
      while (!LL_SPI_IsActiveFlag_RXNE(PORT.channel))
        ;
#endif
      const uint8_t ret = LL_SPI_ReceiveData8(PORT.channel);
      if (rx_data != NULL) {
        rx_data[j] = ret;
      }
    }
  }

#if defined(STM32H7)
  while (!LL_SPI_IsActiveFlag_EOT(PORT.channel))
    ;

  LL_SPI_ClearFlag_TXTF(PORT.channel);
  LL_SPI_Disable(PORT.channel);
#else
  LL_SPI_Disable(PORT.channel);
#endif
  spi_csn_disable(bus);

  dma_transfer_done[port] = 1;
  spi_port_config[port].active_device = NULL;
}

void spi_txn_wait(spi_bus_device_t *bus) {
  while (!spi_txn_ready(bus)) {
    spi_txn_continue(bus);
  }
}

static void spi_txn_finish(spi_bus_device_t *bus) {
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

  dma_free(txn->buffer);

  ATOMIC_BLOCK_ALL {
    txn->buffer = NULL;
    txn->status = TXN_IDLE;

    bus->txns[tail] = NULL;
    bus->txn_tail = tail;

    spi_port_config[bus->port].active_device = NULL;
  }
}

static void handle_dma_rx_isr(spi_ports_t port) {
  const dma_stream_def_t *dma_rx = &dma_stream_defs[PORT.dma_rx];
  const dma_stream_def_t *dma_tx = &dma_stream_defs[PORT.dma_tx];

  if (dma_is_flag_active_te(dma_tx->port, dma_tx->stream_index) || dma_is_flag_active_te(dma_rx->port, dma_rx->stream_index)) {
    failloop(FAILLOOP_SPI);
  }

  if (!dma_is_flag_active_tc(dma_rx->port, dma_rx->stream_index)) {
    return;
  }

  dma_clear_flag_tc(dma_rx->port, dma_rx->stream_index);
  dma_clear_flag_tc(dma_tx->port, dma_tx->stream_index);

  LL_DMA_DisableIT_TC(dma_rx->port, dma_rx->stream_index);
  LL_DMA_DisableIT_TE(dma_rx->port, dma_rx->stream_index);

  LL_SPI_DisableDMAReq_TX(PORT.channel);
  LL_SPI_DisableDMAReq_RX(PORT.channel);

  LL_DMA_DisableStream(dma_rx->port, dma_rx->stream_index);
  LL_DMA_DisableStream(dma_tx->port, dma_tx->stream_index);

#if defined(STM32H7)
  // now we can disable the peripheral
  LL_SPI_ClearFlag_TXTF(PORT.channel);
#endif

  LL_SPI_Disable(PORT.channel);

  if (!spi_port_config[port].active_device) {
    dma_transfer_done[port] = 1;
    return;
  }

  spi_bus_device_t *bus = spi_port_config[port].active_device;
  spi_csn_disable(bus);

  spi_txn_finish(bus);
  dma_transfer_done[port] = 1;

  if (bus->auto_continue) {
    spi_txn_continue(bus);
  }
}

void spi_dma_isr(dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
    handle_dma_rx_isr(SPI_PORT1);
    break;

  case DMA_DEVICE_SPI2_RX:
    handle_dma_rx_isr(SPI_PORT2);
    break;

  case DMA_DEVICE_SPI3_RX:
    handle_dma_rx_isr(SPI_PORT3);
    break;

#if defined(STM32F7) || defined(STM32H7)
  case DMA_DEVICE_SPI4_RX:
    handle_dma_rx_isr(SPI_PORT4);
    break;
#endif

  default:
    break;
  }
}
