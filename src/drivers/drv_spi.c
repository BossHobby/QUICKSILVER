#include "drv_spi.h"

#include <string.h>

#include "project.h"
#include "usb_configurator.h"

#include "drv_dma.h"
#include "drv_interrupt.h"

#define GPIO_AF_SPI1 GPIO_AF5_SPI1
#define GPIO_AF_SPI2 GPIO_AF5_SPI2
#define GPIO_AF_SPI3 GPIO_AF6_SPI3
#define GPIO_AF_SPI4 GPIO_AF5_SPI4
#define GPIO_AF_SPI5 GPIO_AF5_SPI5
#define GPIO_AF_SPI6 GPIO_AF5_SPI6

#define SPI_DMA(spi_prt, dma_prt, chan, rx, tx) \
  {                                             \
    .dma = DMA##dma_prt,                        \
    .dma_port = dma_prt,                        \
    .channel = LL_DMA_CHANNEL_##chan,           \
    .channel_index = chan,                      \
                                                \
    .rx_stream_index = LL_DMA_STREAM_##rx,      \
    .rx_stream = DMA##dma_prt##_Stream##rx,     \
    .rx_it = DMA##dma_prt##_Stream##rx##_IRQn,  \
                                                \
    .tx_stream_index = LL_DMA_STREAM_##tx,      \
    .tx_stream = DMA##dma_prt##_Stream##tx,     \
    .tx_it = DMA##dma_prt##_Stream##tx##_IRQn,  \
  }
#define SPI_PORT(chan, sck_pin, miso_pin, mosi_pin) \
  {                                                 \
      .channel_index = chan,                        \
      .channel = SPI##chan,                         \
      .gpio_af = GPIO_AF_SPI##chan,                 \
      .sck = sck_pin,                               \
      .miso = miso_pin,                             \
      .mosi = mosi_pin,                             \
      .dma = SPI_DMA##chan,                         \
  },

const spi_port_def_t spi_port_defs[SPI_PORTS_MAX] = {{}, SPI_PORTS};

#undef SPI_PORT
#undef SPI_DMA

typedef struct {
  volatile spi_bus_device_t *active_device;
  spi_mode_t mode;
  uint32_t divider;
} spi_port_config_t;

#define SPI_PORT(chan, sck_pin, miso_pin, mosi_pin) \
  {                                                 \
      .active_device = NULL,                        \
      .mode = SPI_MODE_INVALID,                     \
      .divider = 0,                                 \
  },

static volatile spi_port_config_t spi_port_config[SPI_PORTS_MAX] = {{}, SPI_PORTS};

#undef SPI_PORT

#define PORT spi_port_defs[port]

int liberror = 0;

static uint32_t dma_is_flag_active_tc(DMA_TypeDef *dma, uint32_t stream) {
  switch (stream) {
  case LL_DMA_STREAM_0:
    return LL_DMA_IsActiveFlag_TC0(dma);
  case LL_DMA_STREAM_1:
    return LL_DMA_IsActiveFlag_TC1(dma);
  case LL_DMA_STREAM_2:
    return LL_DMA_IsActiveFlag_TC2(dma);
  case LL_DMA_STREAM_3:
    return LL_DMA_IsActiveFlag_TC3(dma);
  case LL_DMA_STREAM_4:
    return LL_DMA_IsActiveFlag_TC4(dma);
  case LL_DMA_STREAM_5:
    return LL_DMA_IsActiveFlag_TC5(dma);
  case LL_DMA_STREAM_6:
    return LL_DMA_IsActiveFlag_TC6(dma);
  case LL_DMA_STREAM_7:
    return LL_DMA_IsActiveFlag_TC7(dma);
  }
  return 0;
}

static void dma_clear_flag_tc(DMA_TypeDef *dma, uint32_t stream) {
  switch (stream) {
  case LL_DMA_STREAM_0:
    LL_DMA_ClearFlag_TC0(dma);
    LL_DMA_ClearFlag_HT0(dma);
    LL_DMA_ClearFlag_FE0(dma);
    break;
  case LL_DMA_STREAM_1:
    LL_DMA_ClearFlag_TC1(dma);
    LL_DMA_ClearFlag_HT1(dma);
    LL_DMA_ClearFlag_FE1(dma);
    break;
  case LL_DMA_STREAM_2:
    LL_DMA_ClearFlag_TC2(dma);
    LL_DMA_ClearFlag_HT2(dma);
    LL_DMA_ClearFlag_FE2(dma);
    break;
  case LL_DMA_STREAM_3:
    LL_DMA_ClearFlag_TC3(dma);
    LL_DMA_ClearFlag_HT3(dma);
    LL_DMA_ClearFlag_FE3(dma);
    break;
  case LL_DMA_STREAM_4:
    LL_DMA_ClearFlag_TC4(dma);
    LL_DMA_ClearFlag_HT4(dma);
    LL_DMA_ClearFlag_FE4(dma);
    break;
  case LL_DMA_STREAM_5:
    LL_DMA_ClearFlag_TC5(dma);
    LL_DMA_ClearFlag_HT5(dma);
    LL_DMA_ClearFlag_FE5(dma);
    break;
  case LL_DMA_STREAM_6:
    LL_DMA_ClearFlag_TC6(dma);
    LL_DMA_ClearFlag_HT6(dma);
    LL_DMA_ClearFlag_FE6(dma);
    break;
  case LL_DMA_STREAM_7:
    LL_DMA_ClearFlag_TC7(dma);
    LL_DMA_ClearFlag_HT7(dma);
    LL_DMA_ClearFlag_FE7(dma);
    break;
  }
}

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
#if defined(STM32F7)
  case 4:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI4);
    break;
#endif
  }
}

static void spi_dma_enable_rcc(spi_ports_t port) {
  switch (PORT.dma.dma_port) {
  case 1:
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    break;
  case 2:
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    break;
  }
}

void spi_csn_enable(gpio_pins_t nss) {
  gpio_pin_reset(nss);
}

void spi_csn_disable(gpio_pins_t nss) {
  gpio_pin_set(nss);
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

uint32_t spi_find_divder(uint32_t clk_hz) {
  uint32_t divider = 2;
  uint32_t clock = SPI_CLOCK_FREQ_HZ / divider;

  while ((clock > clk_hz) && (divider < 256)) {
    divider = divider << 1;
    clock = clock >> 1;
  }

  return spi_divider_to_ll(divider);
}

void spi_init_pins(spi_ports_t port, gpio_pins_t nss) {
  LL_GPIO_InitTypeDef gpio_init;

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_pin_init_af(&gpio_init, PORT.sck, PORT.gpio_af);

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_af(&gpio_init, PORT.miso, PORT.gpio_af);

  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_pin_init_af(&gpio_init, PORT.mosi, PORT.gpio_af);

  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_UP;
  gpio_pin_init(&gpio_init, nss);
  gpio_pin_set(nss);
}

uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data) {
  return spi_transfer_byte_timeout(port, data, 0x400);
}

uint8_t spi_transfer_byte_timeout(spi_ports_t port, uint8_t data, uint32_t timeout_max) {
  for (uint16_t timeout = timeout_max; LL_SPI_IsActiveFlag_TXE(PORT.channel) == RESET; timeout--) {
    if (timeout == 0) {
      // liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  LL_SPI_TransmitData8(PORT.channel, data);

  for (uint16_t timeout = timeout_max; LL_SPI_IsActiveFlag_RXNE(PORT.channel) == RESET; timeout--) {
    if (timeout == 0) {
      // liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  for (uint16_t timeout = timeout_max; LL_SPI_IsActiveFlag_BSY(PORT.channel) == SET; timeout--) {
    if (timeout == 0) {
      // liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  return LL_SPI_ReceiveData8(PORT.channel);
}

volatile uint8_t dma_transfer_done[32] = {[0 ... 31] = 1};
#define DMA_TRANSFER_DONE dma_transfer_done[(PORT.dma.dma_port - 1) * 8 + PORT.dma.rx_stream_index]

void spi_dma_init(spi_ports_t port) {
  // Enable DMA clock
  spi_dma_enable_rcc(port);

  interrupt_enable(PORT.dma.rx_it, DMA_PRIORITY);

  DMA_TRANSFER_DONE = 1;
}

static void spi_dma_receive_init(spi_ports_t port, uint8_t *base_address_in, uint32_t buffer_size) {
  //RX Stream
  LL_DMA_DeInit(PORT.dma.dma, PORT.dma.rx_stream_index);

  dma_prepare_rx_memory(base_address_in, buffer_size);

  LL_DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.Channel = PORT.dma.channel;
  DMA_InitStructure.PeriphOrM2MSrcAddress = LL_SPI_DMA_GetRegAddr(PORT.channel);
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)base_address_in;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.NbData = (uint16_t)buffer_size;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(PORT.dma.dma, PORT.dma.rx_stream_index, &DMA_InitStructure);
}

static void spi_dma_transmit_init(spi_ports_t port, uint8_t *base_address_out, uint32_t buffer_size) {
  //TX Stream
  LL_DMA_DeInit(PORT.dma.dma, PORT.dma.tx_stream_index);

  dma_prepare_tx_memory(base_address_out, buffer_size);

  LL_DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.Channel = PORT.dma.channel;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)(&(PORT.channel->DR));
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)base_address_out;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.NbData = (uint16_t)buffer_size;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(PORT.dma.dma, PORT.dma.tx_stream_index, &DMA_InitStructure);
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
  return DMA_TRANSFER_DONE;
}

bool spi_dma_wait_for_ready(spi_ports_t port) {
  for (uint16_t timeout = 0x400; spi_dma_is_ready(port) == 0; timeout--) {
    if (timeout == 0) {
      // liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return false;
    }
    __WFI();
  }
  return true;
}

void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  if (!spi_dma_wait_for_ready(port)) {
    return;
  }
  DMA_TRANSFER_DONE = 0;

  spi_dma_receive_init(port, buffer, length);
  spi_dma_transmit_init(port, buffer, length);

  // Enable the SPI Rx/Tx DMA request
  LL_SPI_EnableDMAReq_TX(PORT.channel);
  LL_SPI_EnableDMAReq_RX(PORT.channel);

  // if everything goes right, those are not required.
  // DMA_ClearFlag(PORT.dma.rx_stream, PORT.dma.rx_tci_flag);
  // DMA_ClearFlag(PORT.dma.tx_stream, PORT.dma.tx_tci_flag);

  // DMA_ClearITPendingBit(PORT.dma.rx_stream, PORT.dma.rx_it_flag);
  LL_DMA_EnableIT_TC(PORT.dma.dma, PORT.dma.rx_stream_index);

  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

  // now we can enable the peripheral
  // LL_SPI_Enable(PORT.channel);
}

// blocking dma transmit bytes
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  if (!spi_dma_wait_for_ready(port)) {
    return;
  }
  DMA_TRANSFER_DONE = 0;

  spi_dma_receive_init(port, buffer, length);
  spi_dma_transmit_init(port, buffer, length);

  // Enable the SPI Rx/Tx DMA request
  LL_SPI_EnableDMAReq_TX(PORT.channel);
  LL_SPI_EnableDMAReq_RX(PORT.channel);

  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

  while (dma_is_flag_active_tc(PORT.dma.dma, PORT.dma.tx_stream_index) == RESET)
    ;
  while (dma_is_flag_active_tc(PORT.dma.dma, PORT.dma.rx_stream_index) == RESET)
    ;

  dma_clear_flag_tc(PORT.dma.dma, PORT.dma.rx_stream_index);
  dma_clear_flag_tc(PORT.dma.dma, PORT.dma.tx_stream_index);

  LL_SPI_DisableDMAReq_TX(PORT.channel);
  LL_SPI_DisableDMAReq_RX(PORT.channel);

  LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
  LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

  DMA_TRANSFER_DONE = 1;
}

void spi_bus_device_init(volatile spi_bus_device_t *bus) {
  bus->txn_head = 0;
  bus->txn_tail = 0;

  spi_init_pins(bus->port, bus->nss);
  spi_enable_rcc(bus->port);

  LL_SPI_DeInit(spi_port_defs[bus->port].channel);

  LL_SPI_InitTypeDef default_init;
  default_init.TransferDirection = LL_SPI_FULL_DUPLEX;
  default_init.Mode = LL_SPI_MODE_MASTER;
  default_init.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  default_init.ClockPolarity = LL_SPI_POLARITY_LOW;
  default_init.ClockPhase = LL_SPI_PHASE_1EDGE;
  default_init.NSS = LL_SPI_NSS_SOFT;
  default_init.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
  default_init.BitOrder = LL_SPI_MSB_FIRST;
  default_init.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  default_init.CRCPoly = 7;
  LL_SPI_Init(spi_port_defs[bus->port].channel, &default_init);

  LL_SPI_Enable(spi_port_defs[bus->port].channel);

  // Dummy read to clear receive buffer
  while (LL_SPI_IsActiveFlag_TXE(spi_port_defs[bus->port].channel) == RESET)
    ;
  LL_SPI_ReceiveData8(spi_port_defs[bus->port].channel);

  spi_dma_init(bus->port);
}

void spi_bus_device_reconfigure(volatile spi_bus_device_t *bus, spi_mode_t mode, uint32_t divider) {
  if (spi_port_config[bus->port].mode == mode && spi_port_config[bus->port].divider == divider) {
    return;
  }
  spi_port_config[bus->port].mode = mode;
  spi_port_config[bus->port].divider = divider;

  spi_dma_wait_for_ready(bus->port);

  LL_SPI_Disable(spi_port_defs[bus->port].channel);
  LL_SPI_SetBaudRatePrescaler(spi_port_defs[bus->port].channel, divider);
  if (mode == SPI_MODE_LEADING_EDGE) {
    LL_SPI_SetClockPhase(spi_port_defs[bus->port].channel, LL_SPI_PHASE_1EDGE);
    LL_SPI_SetClockPolarity(spi_port_defs[bus->port].channel, LL_SPI_POLARITY_LOW);
  } else if (mode == SPI_MODE_TRAILING_EDGE) {
    LL_SPI_SetClockPhase(spi_port_defs[bus->port].channel, LL_SPI_PHASE_2EDGE);
    LL_SPI_SetClockPolarity(spi_port_defs[bus->port].channel, LL_SPI_POLARITY_HIGH);
  }
  LL_SPI_Enable(spi_port_defs[bus->port].channel);
}

spi_txn_t *spi_txn_init(volatile spi_bus_device_t *bus, spi_txn_done_fn_t done_fn) {
  bus->txn_head = (bus->txn_head + 1) % SPI_TXN_MAX;

  volatile spi_txn_t *txn = &bus->txns[bus->txn_head];
  txn->bus = bus;
  txn->status = TXN_WAITING;
  txn->segment_count = 0;

  const uint32_t last = bus->txn_head == 0 ? (SPI_TXN_MAX - 1) : bus->txn_head - 1;
  if (last != bus->txn_tail) {
    txn->offset = bus->txns[last].offset + bus->txns[last].size;
  } else {
    txn->offset = 0;
  }

  txn->size = 0;
  txn->done_fn = done_fn;
  return (spi_txn_t *)txn;
}

void spi_txn_add_seg_delay(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size) {
  txn->size += size;

  txn->segments[txn->segment_count].live = true;
  txn->segments[txn->segment_count].rx_data = rx_data;
  txn->segments[txn->segment_count].tx_data = tx_data;
  txn->segments[txn->segment_count].size = size;
  txn->segment_count++;
}

void spi_txn_add_seg(spi_txn_t *txn, uint8_t *rx_data, const uint8_t *tx_data, uint32_t size) {
  if (tx_data) {
    memcpy((uint8_t *)txn->bus->buffer + txn->offset + txn->size, tx_data, size);
  } else {
    memset((uint8_t *)txn->bus->buffer + txn->offset + txn->size, 0xFF, size);
  }
  txn->size += size;

  txn->segments[txn->segment_count].live = false;
  txn->segments[txn->segment_count].rx_data = rx_data;
  txn->segments[txn->segment_count].tx_data = tx_data;
  txn->segments[txn->segment_count].size = size;
  txn->segment_count++;
}

void spi_txn_add_seg_const(spi_txn_t *txn, const uint8_t tx_data) {
  spi_txn_add_seg(txn, NULL, &tx_data, 1);
}

void spi_txn_submit(spi_txn_t *txn) {
  txn->status = TXN_READY;
}

void spi_txn_continue(volatile spi_bus_device_t *bus) {
  if (bus->txn_head == bus->txn_tail) {
    return;
  }
  if (!spi_dma_is_ready(bus->port)) {
    return;
  }
  if (spi_port_config[bus->port].active_device != NULL &&
      spi_port_config[bus->port].active_device != bus) {
    return;
  }
  if (bus->poll_fn && !bus->poll_fn()) {
    return;
  }

  const uint32_t tail = (bus->txn_tail + 1) % SPI_TXN_MAX;

  volatile spi_txn_t *txn = &bus->txns[tail];
  if (txn->status != TXN_READY) {
    return;
  }

  spi_port_config[bus->port].active_device = bus;
  txn->status = TXN_IN_PROGRESS;

  uint32_t txn_size = 0;
  for (uint32_t i = 0; i < txn->segment_count; ++i) {
    volatile spi_txn_segment_t *seg = &txn->segments[i];
    if (seg->live && seg->tx_data) {
      memcpy((uint8_t *)txn->bus->buffer + txn->offset + txn_size, seg->tx_data, seg->size);
    }
    txn_size += seg->size;
  }

  spi_csn_enable(bus->nss);
  spi_dma_transfer_begin(bus->port, (uint8_t *)bus->buffer + txn->offset, txn->size);
}

bool spi_txn_ready(volatile spi_bus_device_t *bus) {
  return bus->txn_head == bus->txn_tail;
}

void spi_txn_wait(volatile spi_bus_device_t *bus) {
  spi_txn_continue(bus);
  while (!spi_txn_ready(bus)) {
    __WFI();
  }
}

static void spi_txn_dma_rx_isr(spi_ports_t port) {
  volatile spi_bus_device_t *bus = spi_port_config[port].active_device;
  spi_csn_disable(bus->nss);

  const uint32_t tail = (bus->txn_tail + 1) % SPI_TXN_MAX;
  volatile spi_txn_t *txn = &bus->txns[tail];

  uint32_t txn_size = 0;
  for (uint32_t i = 0; i < txn->segment_count; ++i) {
    volatile spi_txn_segment_t *seg = &txn->segments[i];
    if (seg->rx_data) {
      memcpy(seg->rx_data, (uint8_t *)txn->bus->buffer + txn->offset + txn_size, seg->size);
    }
    txn_size += seg->size;
  }

  spi_port_config[port].active_device = NULL;
  txn->status = TXN_DONE;
  bus->txn_tail = tail;
  DMA_TRANSFER_DONE = 1;

  if (txn->done_fn) {
    txn->done_fn();
  }

  if (txn->bus->auto_continue) {
    spi_txn_continue(bus);
  }
}

static void handle_dma_rx_isr(spi_ports_t port) {
  if (!dma_is_flag_active_tc(PORT.dma.dma, PORT.dma.rx_stream_index)) {
    return;
  }

  dma_clear_flag_tc(PORT.dma.dma, PORT.dma.rx_stream_index);
  dma_clear_flag_tc(PORT.dma.dma, PORT.dma.tx_stream_index);

  LL_DMA_DisableIT_TC(PORT.dma.dma, PORT.dma.rx_stream_index);

  LL_SPI_DisableDMAReq_TX(PORT.channel);
  LL_SPI_DisableDMAReq_RX(PORT.channel);

  LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
  LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

  // now we can disable the peripheral
  //LL_SPI_Disable(PORT.channel);

  if (spi_port_config[port].active_device) {
    spi_txn_dma_rx_isr(port);
  } else {
#if defined(USE_SDCARD) && defined(SDCARD_SPI_PORT)
    if (port == SDCARD_SPI_PORT) {
      extern void sdcard_dma_rx_isr();
      sdcard_dma_rx_isr();
    }
#endif

#if defined(USE_M25P16) && defined(M25P16_SPI_PORT)
    if (port == M25P16_SPI_PORT) {
      extern void m25p16_dma_rx_isr();
      m25p16_dma_rx_isr();
    }
#endif

    DMA_TRANSFER_DONE = 1;
  }
}

#define SPI_PORT(channel, sck_pin, miso_pin, mosi_pin) SPI_DMA##channel
#define SPI_DMA(spi_prt, dma_prt, chan, rx, tx)   \
  void DMA##dma_prt##_Stream##rx##_IRQHandler() { \
    handle_dma_rx_isr(SPI_PORT##spi_prt);         \
  }

SPI_PORTS

#undef SPI_DMA
#undef SPI_PORT
