#include "drv_spi.h"

#include <string.h>

#include "failloop.h"
#include "io/usb_configurator.h"
#include "project.h"

#include "drv_dma.h"
#include "drv_interrupt.h"
#include "drv_time.h"

#define GPIO_AF_SPI1 GPIO_AF5_SPI1
#define GPIO_AF_SPI2 GPIO_AF5_SPI2
#define GPIO_AF_SPI3 GPIO_AF6_SPI3
#define GPIO_AF_SPI4 GPIO_AF5_SPI4
#define GPIO_AF_SPI5 GPIO_AF5_SPI5
#define GPIO_AF_SPI6 GPIO_AF5_SPI6

#define SPI_PORT(chan, sck_pin, miso_pin, mosi_pin) \
  {                                                 \
      .channel_index = chan,                        \
      .channel = SPI##chan,                         \
      .gpio_af = GPIO_AF_SPI##chan,                 \
      .sck = sck_pin,                               \
      .miso = miso_pin,                             \
      .mosi = mosi_pin,                             \
      .dma_rx = DMA_DEVICE_SPI##chan##_RX,          \
      .dma_tx = DMA_DEVICE_SPI##chan##_TX,          \
  },

const spi_port_def_t spi_port_defs[SPI_PORTS_MAX] = {{}, SPI_PORTS};

#undef SPI_PORT

typedef struct {
  spi_bus_device_t *active_device;
  spi_mode_t mode;
  uint32_t hz;
} spi_port_config_t;

#define SPI_PORT(chan, sck_pin, miso_pin, mosi_pin) \
  {                                                 \
      .active_device = NULL,                        \
      .mode = SPI_MODE_INVALID,                     \
      .hz = 0,                                      \
  },

static volatile spi_port_config_t spi_port_config[SPI_PORTS_MAX] = {{}, SPI_PORTS};

#undef SPI_PORT

volatile uint8_t dma_transfer_done[16] = {[0 ... 15] = 1};

#define PORT spi_port_defs[port]

void spi_enable_rcc(spi_ports_t port) {
  switch (PORT.channel_index) {
  case 1:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    break;
  case 2:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    break;
  case 3:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3);
    break;
#if defined(STM32F7) || defined(STM32H7)
  case 4:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
    break;
#endif
  }
}

void spi_csn_enable(spi_bus_device_t *bus) {
  gpio_pin_reset(bus->nss);
}

void spi_csn_disable(spi_bus_device_t *bus) {
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
  LL_GPIO_InitTypeDef gpio_init;

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_pin_init_af(&gpio_init, PORT.sck, PORT.gpio_af);

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_af(&gpio_init, PORT.miso, PORT.gpio_af);

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_af(&gpio_init, PORT.mosi, PORT.gpio_af);

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

uint8_t spi_dma_is_ready(spi_ports_t port) {
#if defined(BRUSHLESS_TARGET) && defined(STM32F4)
  if (port == SPI_PORT1) {
    extern volatile int dshot_dma_phase;
    if (dshot_dma_phase != 0) {
      return 0;
    }
  }
#endif
  return dma_transfer_done[port];
}

static void spi_reconfigure(spi_bus_device_t *bus, volatile spi_port_config_t *config) {
  const spi_port_def_t *port = &spi_port_defs[bus->port];

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
      gpio_pin_init_af(&gpio_init, port->sck, port->gpio_af);

      LL_SPI_SetClockPhase(port->channel, LL_SPI_PHASE_1EDGE);
      LL_SPI_SetClockPolarity(port->channel, LL_SPI_POLARITY_LOW);
    } else if (bus->mode == SPI_MODE_TRAILING_EDGE) {
      gpio_init.Pull = LL_GPIO_PULL_UP;
      gpio_pin_init_af(&gpio_init, port->sck, port->gpio_af);

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

static uint8_t spi_transfer(spi_ports_t port, uint8_t *data, uint32_t size) {
#if defined(STM32H7)
  LL_SPI_SetTransferSize(PORT.channel, size);
  LL_SPI_Enable(PORT.channel);
  LL_SPI_StartMasterTransfer(PORT.channel);

  for (uint32_t i = 0; i < size; i++) {
    while (!LL_SPI_IsActiveFlag_TXP(PORT.channel))
      ;

    LL_SPI_TransmitData8(PORT.channel, data[i]);

    while (!LL_SPI_IsActiveFlag_RXP(PORT.channel))
      ;

    data[i] = LL_SPI_ReceiveData8(PORT.channel);
  }

  while (!LL_SPI_IsActiveFlag_EOT(PORT.channel))
    ;

  LL_SPI_ClearFlag_TXTF(PORT.channel);
  LL_SPI_Disable(PORT.channel);
#else
  LL_SPI_Enable(PORT.channel);

  for (uint32_t i = 0; i < size; i++) {
    while (LL_SPI_IsActiveFlag_BSY(PORT.channel) || !LL_SPI_IsActiveFlag_TXE(PORT.channel))
      ;

    LL_SPI_TransmitData8(PORT.channel, data[i]);

    while (LL_SPI_IsActiveFlag_BSY(PORT.channel) || !LL_SPI_IsActiveFlag_RXNE(PORT.channel))
      ;

    data[i] = LL_SPI_ReceiveData8(PORT.channel);
  }

  LL_SPI_Disable(PORT.channel);
#endif

  return 1;
}

void spi_bus_device_init(spi_bus_device_t *bus) {
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

spi_txn_t *spi_txn_init(spi_bus_device_t *bus, spi_txn_done_fn_t done_fn) {
  spi_txn_t *txn = spi_txn_pop(bus);
  if (txn == NULL) {
    failloop(FAILLOOP_SPI);
  }

  txn->buffer = dma_alloc(16);
  txn->buffer_size = 16;

  txn->bus = bus;
  txn->segment_count = 0;

  txn->flags = 0;
  txn->size = 0;
  txn->done_fn = done_fn;

  return (spi_txn_t *)txn;
}

static void spi_txn_ensure_buffer_space(spi_txn_t *txn, uint32_t size) {
  if (txn->buffer_size >= (txn->size + size)) {
    return;
  }

  uint32_t new_size = txn->buffer_size;
  while (new_size < (txn->size + size)) {
    new_size *= 2;
  }

  txn->buffer = dma_realloc(txn->buffer, new_size);
  txn->buffer_size = new_size;
}

void spi_txn_add_seg_delay(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size) {
  if (size == 0) {
    return;
  }

  spi_txn_ensure_buffer_space(txn, size);

  txn->size += size;
  txn->flags |= TXN_DELAYED_TX;

  txn->segments[txn->segment_count].rx_data = rx_data;
  txn->segments[txn->segment_count].tx_data = tx_data;
  txn->segments[txn->segment_count].size = size;
  txn->segment_count++;
}

void spi_txn_add_seg(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size) {
  if (size == 0) {
    return;
  }

  spi_txn_ensure_buffer_space(txn, size);

  for (uint32_t i = 0; i < size; i++) {
    txn->buffer[txn->size + i] = tx_data ? tx_data[i] : 0xFF;
  }
  txn->size += size;

  spi_txn_segment_t *last_seg = txn->segment_count > 0 ? &txn->segments[txn->segment_count - 1] : NULL;
  if (rx_data == NULL && last_seg != NULL && last_seg->rx_data == NULL && last_seg->tx_data == NULL) {
    // merge segments
    last_seg->size += size;
  } else {
    // create new segment
    if (txn->segment_count >= SPI_TXN_SEG_MAX) {
      failloop(FAILLOOP_SPI);
    }

    if (rx_data) {
      txn->flags |= TXN_DELAYED_RX;
    }

    spi_txn_segment_t *seg = &txn->segments[txn->segment_count];
    seg->rx_data = rx_data;
    seg->tx_data = NULL;
    seg->size = size;
    txn->segment_count++;
  }
}

void spi_txn_add_seg_const(spi_txn_t *txn, const uint8_t tx_data) {
  spi_txn_add_seg(txn, NULL, &tx_data, 1);
}

bool spi_txn_ready(spi_bus_device_t *bus) {
  return bus->txn_head == bus->txn_tail;
}

void spi_txn_submit(spi_txn_t *txn) {
  ATOMIC_BLOCK_ALL {
    const uint8_t head = (txn->bus->txn_head + 1) % SPI_TXN_MAX;
    txn->status = TXN_READY;
    txn->bus->txns[head] = txn;
    txn->bus->txn_head = head;
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
  txn->buffer = NULL;
  txn->status = TXN_IDLE;

  bus->txns[tail] = NULL;
  bus->txn_tail = tail;

  spi_port_config[bus->port].active_device = NULL;
}

static bool spi_txn_should_use_dma(spi_bus_device_t *bus, spi_txn_t *txn) {
#ifdef STM32H7
  if (WITHIN_DTCM_RAM(txn->buffer) || !WITHIN_DMA_RAM(txn->buffer)) {
    return false;
  }
#endif
  return true;
}

void spi_txn_continue_ex(spi_bus_device_t *bus, bool force_sync) {
  ATOMIC_BLOCK_ALL {
    if (!spi_dma_is_ready(bus->port)) {
      return;
    }

    if (bus->txn_head == bus->txn_tail) {
      return;
    }

    volatile spi_port_config_t *config = &spi_port_config[bus->port];
    if (config->active_device != NULL && config->active_device != bus) {
      return;
    }

    if (bus->poll_fn && !bus->poll_fn()) {
      return;
    }

    const uint32_t tail = (bus->txn_tail + 1) % SPI_TXN_MAX;
    spi_txn_t *txn = bus->txns[tail];

    config->active_device = bus;
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

    spi_reconfigure(bus, config);

    if (spi_txn_should_use_dma(bus, txn) && !force_sync) {
      spi_csn_enable(bus);
      spi_dma_transfer_begin(bus->port, txn->buffer, txn->size);
    } else {
      spi_csn_enable(bus);
      spi_transfer(bus->port, txn->buffer, txn->size);
      spi_csn_disable(bus);

      spi_txn_finish(bus);

      if (bus->auto_continue) {
        spi_txn_continue(bus);
      }
    }
  }
}

void spi_txn_continue(spi_bus_device_t *bus) {
  spi_txn_continue_ex(bus, false);
}

void spi_txn_wait(spi_bus_device_t *bus) {
  while (!spi_txn_ready(bus)) {
    spi_txn_continue(bus);
  }
}

void spi_txn_submit_wait(spi_bus_device_t *bus, spi_txn_t *txn) {
  spi_txn_submit(txn);
  spi_txn_wait(bus);
}

void spi_txn_submit_continue(spi_bus_device_t *bus, spi_txn_t *txn) {
  spi_txn_submit(txn);
  spi_txn_continue(bus);
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

#define SPI_PORT(channel, sck_pin, miso_pin, mosi_pin) \
  case DMA_DEVICE_SPI##channel##_RX:                   \
    handle_dma_rx_isr(SPI_PORT##channel);              \
    break;                                             \
  case DMA_DEVICE_SPI##channel##_TX:                   \
    break;

void spi_dma_isr(dma_device_t dev) {
  switch (dev) {
    SPI_PORTS

  default:
    break;
  }
}

#undef SPI_DMA