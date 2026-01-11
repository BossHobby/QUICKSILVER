#include "driver/spi.h"

#include "core/failloop.h"
#include "driver/interrupt.h"

extern bool spi_txn_can_send(spi_bus_device_t *bus, bool dma);
extern void spi_txn_finish(spi_ports_t port);

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

extern FAST_RAM spi_txn_t txn_pool[SPI_TXN_MAX];

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

static void spi_dma_init_rx(spi_ports_t port) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[spi_port_defs[port].dma_rx].dma];

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);
#if defined(STM32H7) || defined(STM32G4)
  dma_init.PeriphRequest = target.dma[spi_port_defs[port].dma_rx].request;
#else
  dma_init.Channel = dma_map_channel(target.dma[spi_port_defs[port].dma_rx].channel);
#endif
#if defined(STM32H7)
  dma_init.PeriphOrM2MSrcAddress = (uint32_t)&spi_port_defs[port].channel->RXDR;
#else
  dma_init.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(spi_port_defs[port].channel);
#endif
  dma_init.MemoryOrM2MDstAddress = 0;
  dma_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.NbData = 0;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.Priority = LL_DMA_PRIORITY_HIGH;
#ifndef STM32G4
  dma_init.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
  dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(dma->port, dma->stream_index, &dma_init);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  interrupt_enable(dma->irq, DMA_PRIORITY);
  dma_clear_flag_tc(dma);
  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);
  LL_DMA_EnableIT_TE(dma->port, dma->stream_index);
}

static void spi_dma_init_tx(spi_ports_t port) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[spi_port_defs[port].dma_tx].dma];

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef dma_init;
  LL_DMA_StructInit(&dma_init);
#if defined(STM32H7) || defined(STM32G4)
  dma_init.PeriphRequest = target.dma[spi_port_defs[port].dma_tx].request;
#else
  dma_init.Channel = dma_map_channel(target.dma[spi_port_defs[port].dma_tx].channel);
#endif
#if defined(STM32H7)
  dma_init.PeriphOrM2MSrcAddress = (uint32_t)&spi_port_defs[port].channel->TXDR;
#else
  dma_init.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(spi_port_defs[port].channel);
#endif
  dma_init.MemoryOrM2MDstAddress = 0;
  dma_init.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  dma_init.NbData = 0;
  dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  dma_init.Mode = LL_DMA_MODE_NORMAL;
  dma_init.Priority = LL_DMA_PRIORITY_VERYHIGH;
#ifndef STM32G4
  dma_init.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  dma_init.MemBurst = LL_DMA_MBURST_SINGLE;
  dma_init.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(dma->port, dma->stream_index, &dma_init);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  dma_clear_flag_tc(dma);
  interrupt_enable(dma->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);
  LL_DMA_EnableIT_TE(dma->port, dma->stream_index);
}

void spi_reconfigure(spi_bus_device_t *bus) {
  spi_device_t *config = &spi_dev[bus->port];
  const spi_port_def_t *port = &spi_port_defs[bus->port];

  if (config->hz != bus->hz) {
    config->hz = bus->hz;
    LL_SPI_SetBaudRatePrescaler(port->channel, spi_find_divder(bus->hz));
  }
  if (config->mode != bus->mode) {
    config->mode = bus->mode;

    gpio_config_t gpio_init = gpio_config_default();
    gpio_init.mode = GPIO_ALTERNATE;
    gpio_init.drive = GPIO_DRIVE_HIGH;
    gpio_init.output = GPIO_PUSHPULL;

    const target_spi_port_t *dev = &target.spi_ports[bus->port];
    if (bus->mode == SPI_MODE_LEADING_EDGE) {
      gpio_init.pull = GPIO_DOWN_PULL;
      gpio_pin_init_tag(dev->sck, gpio_init, SPI_TAG(bus->port, RES_SPI_SCK));

      LL_SPI_SetClockPhase(port->channel, LL_SPI_PHASE_1EDGE);
      LL_SPI_SetClockPolarity(port->channel, LL_SPI_POLARITY_LOW);
    } else if (bus->mode == SPI_MODE_TRAILING_EDGE) {
      gpio_init.pull = GPIO_UP_PULL;
      gpio_pin_init_tag(dev->sck, gpio_init, SPI_TAG(bus->port, RES_SPI_SCK));

      LL_SPI_SetClockPhase(port->channel, LL_SPI_PHASE_2EDGE);
      LL_SPI_SetClockPolarity(port->channel, LL_SPI_POLARITY_HIGH);
    }
  }
}

void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length, bool has_rx) {
#if !defined(STM32H7)
  // dummy read
  while (LL_SPI_IsActiveFlag_RXNE(spi_port_defs[port].channel) || LL_SPI_IsActiveFlag_OVR(spi_port_defs[port].channel))
    LL_SPI_ReceiveData8(spi_port_defs[port].channel);
#endif

  spi_dev[port].dma_rx_done = !has_rx;
  spi_dev[port].dma_tx_done = false;

  const dma_stream_def_t *dma_tx = &dma_stream_defs[target.dma[spi_port_defs[port].dma_tx].dma];
  dma_clear_flag_tc(dma_tx);
  dma_prepare_tx_memory(buffer, length);

  while (LL_DMA_IsEnabledStream(dma_tx->port, dma_tx->stream_index))
    ;

  LL_DMA_SetMemoryAddress(dma_tx->port, dma_tx->stream_index, (uint32_t)buffer);
  LL_DMA_SetDataLength(dma_tx->port, dma_tx->stream_index, length);
  LL_DMA_EnableStream(dma_tx->port, dma_tx->stream_index);
  LL_SPI_EnableDMAReq_TX(spi_port_defs[port].channel);

  if (has_rx) {
    const dma_stream_def_t *dma_rx = &dma_stream_defs[target.dma[spi_port_defs[port].dma_rx].dma];
    dma_clear_flag_tc(dma_rx);
    dma_prepare_rx_memory(buffer, length);

    while (LL_DMA_IsEnabledStream(dma_rx->port, dma_rx->stream_index))
      ;

    LL_DMA_SetMemoryAddress(dma_rx->port, dma_rx->stream_index, (uint32_t)buffer);
    LL_DMA_SetDataLength(dma_rx->port, dma_rx->stream_index, length);
    LL_DMA_EnableStream(dma_rx->port, dma_rx->stream_index);
    LL_SPI_EnableDMAReq_RX(spi_port_defs[port].channel);
  }

#ifdef STM32H7
  LL_SPI_SetTransferSize(spi_port_defs[port].channel, length);
#endif
  LL_SPI_Enable(spi_port_defs[port].channel);
#ifdef STM32H7
  LL_SPI_StartMasterTransfer(spi_port_defs[port].channel);
#endif
}

void spi_device_init(spi_ports_t port) {
  const spi_port_def_t *def = &spi_port_defs[port];
  rcc_enable(def->rcc);

  const dma_stream_def_t *dma_rx = &dma_stream_defs[target.dma[spi_port_defs[port].dma_rx].dma];
  const dma_stream_def_t *dma_tx = &dma_stream_defs[target.dma[spi_port_defs[port].dma_tx].dma];

  dma_enable_rcc(dma_rx);
  dma_enable_rcc(dma_tx);

  LL_SPI_DeInit(def->channel);

  LL_SPI_InitTypeDef default_init;
  LL_SPI_StructInit(&default_init);
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
  LL_SPI_EnableGPIOControl(def->channel);
  LL_SPI_SetFIFOThreshold(def->channel, LL_SPI_FIFO_TH_01DATA);
#endif
  LL_SPI_Init(def->channel, &default_init);
#if defined(STM32G4)
  LL_SPI_SetRxFIFOThreshold(def->channel, LL_SPI_RX_FIFO_TH_QUARTER);
  LL_SPI_SetStandard(def->channel, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_DisableNSSPulseMgt(def->channel);
#endif

  spi_dev[port].dma_done = true;
  spi_dev[port].mode = SPI_MODE_TRAILING_EDGE;
  spi_dev[port].hz = 0;

  spi_dma_init_rx(port);
  spi_dma_init_tx(port);
}

static inline void spi_put(const spi_ports_t port, const uint8_t data) {
#if defined(STM32H7)
  while (!LL_SPI_IsActiveFlag_TXP(spi_port_defs[port].channel))
    ;
#else
  while (!LL_SPI_IsActiveFlag_TXE(spi_port_defs[port].channel))
    ;
#endif
  LL_SPI_TransmitData8(spi_port_defs[port].channel, data);
}

static inline uint8_t spi_get(const spi_ports_t port) {
#if defined(STM32H7)
  while (!LL_SPI_IsActiveFlag_RXP(spi_port_defs[port].channel))
    ;
#else
  while (!LL_SPI_IsActiveFlag_RXNE(spi_port_defs[port].channel))
    ;
#endif
  return LL_SPI_ReceiveData8(spi_port_defs[port].channel);
}

void spi_seg_submit_wait_ex(spi_bus_device_t *bus, const spi_txn_segment_t *segs, const uint32_t count) {
  spi_txn_wait(bus);

  while (!spi_txn_can_send(bus, false))
    ;

  const spi_ports_t port = bus->port;

  spi_dev[port].dma_done = false;

  spi_reconfigure(bus);

  spi_csn_enable(bus);
#if defined(STM32H7)
  uint32_t transfer_size = 0;
  for (uint32_t i = 0; i < count; i++) {
    transfer_size += segs[i].size;
  }
  LL_SPI_SetTransferSize(spi_port_defs[port].channel, transfer_size);
  LL_SPI_Enable(spi_port_defs[port].channel);
  LL_SPI_StartMasterTransfer(spi_port_defs[port].channel);
#else
  LL_SPI_Enable(spi_port_defs[port].channel);
#endif

  for (uint32_t i = 0; i < count; i++) {
    const spi_txn_segment_t *seg = &segs[i];
    const uint32_t size = seg->size;

    const uint8_t *tx_data = NULL;
    uint8_t *rx_data = NULL;

    if (seg->type == TXN_CONST) {
      tx_data = seg->bytes;
    } else {
      tx_data = seg->tx_data;
      rx_data = seg->rx_data;
    }

    if (tx_data && rx_data) {
      spi_put(port, *tx_data++);
      for (uint32_t j = 1; j < size; j++) {
        spi_put(port, *tx_data++);
        *rx_data++ = spi_get(port);
      }
      *rx_data = spi_get(port);
    } else if (tx_data) {
      spi_put(port, *tx_data++);
      for (uint32_t j = 1; j < size; j++) {
        spi_put(port, *tx_data++);
        spi_get(port);
      }
      spi_get(port);
    } else if (rx_data) {
      spi_put(port, 0xFF);
      for (uint32_t j = 1; j < size; j++) {
        spi_put(port, 0xFF);
        *rx_data++ = spi_get(port);
      }
      *rx_data = spi_get(port);
    }
  }

#if defined(STM32H7)
  while (!LL_SPI_IsActiveFlag_EOT(spi_port_defs[port].channel))
    ;

  LL_SPI_ClearFlag_TXTF(spi_port_defs[port].channel);
  LL_SPI_Disable(spi_port_defs[port].channel);
#else
  LL_SPI_Disable(spi_port_defs[port].channel);
#endif
  spi_csn_disable(bus);

  spi_dev[port].dma_done = true;
}

static void txn_finish(spi_ports_t port) {
  // read to clear overrun flag and drain RX FIFO
#ifdef STM32G4
  while (LL_SPI_IsActiveFlag_RXNE(spi_port_defs[port].channel) || LL_SPI_IsActiveFlag_OVR(spi_port_defs[port].channel))
    LL_SPI_ReceiveData8(spi_port_defs[port].channel);
#else
  LL_SPI_ReceiveData8(spi_port_defs[port].channel);
#endif

#if defined(STM32H7)
  // now we can disable the peripheral
  LL_SPI_ClearFlag_TXTF(spi_port_defs[port].channel);
#endif

  LL_SPI_Disable(spi_port_defs[port].channel);
  spi_txn_finish(port);
}

static void handle_dma_rx_isr(spi_ports_t port) {
  const spi_port_def_t *def = &spi_port_defs[port];
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[def->dma_rx].dma];
  if (!dma_is_flag_active(dma, DMA_FLAG_TC))
    return;

  dma_clear_flag_tc(dma);
  LL_SPI_DisableDMAReq_RX(def->channel);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  spi_dev[port].dma_rx_done = true;

  if (spi_dev[port].dma_tx_done)
    txn_finish(port);
}

static void handle_dma_tx_isr(spi_ports_t port) {
  const spi_port_def_t *def = &spi_port_defs[port];
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[def->dma_tx].dma];
  if (!dma_is_flag_active(dma, DMA_FLAG_TC))
    return;

  dma_clear_flag_tc(dma);
  LL_SPI_DisableDMAReq_TX(def->channel);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  spi_dev[port].dma_tx_done = true;

  if (spi_dev[port].dma_rx_done)
    txn_finish(port);
}

void spi_dma_isr(const dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
    handle_dma_rx_isr(SPI_PORT1);
    break;
  case DMA_DEVICE_SPI1_TX:
    handle_dma_tx_isr(SPI_PORT1);
    break;

  case DMA_DEVICE_SPI2_RX:
    handle_dma_rx_isr(SPI_PORT2);
    break;
  case DMA_DEVICE_SPI2_TX:
    handle_dma_tx_isr(SPI_PORT2);
    break;

  case DMA_DEVICE_SPI3_RX:
    handle_dma_rx_isr(SPI_PORT3);
    break;
  case DMA_DEVICE_SPI3_TX:
    handle_dma_tx_isr(SPI_PORT3);
    break;

#if defined(STM32F7) || defined(STM32H7)
  case DMA_DEVICE_SPI4_RX:
    handle_dma_rx_isr(SPI_PORT4);
    break;
  case DMA_DEVICE_SPI4_TX:
    handle_dma_tx_isr(SPI_PORT4);
    break;
#endif

  default:
    break;
  }
}
