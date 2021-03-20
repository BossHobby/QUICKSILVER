#include "drv_spi.h"

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_spi.h>

#include "usb_configurator.h"

#define GPIO_AF_SPI1 GPIO_AF5_SPI1
#define GPIO_AF_SPI2 GPIO_AF5_SPI2
#define GPIO_AF_SPI3 GPIO_AF6_SPI3

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

const volatile spi_port_def_t spi_port_defs[SPI_PORTS_MAX] = {
    {},
    SPI_PORTS};

#undef SPI_PORT
#undef SPI_DMA

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
  }
}

void spi_dma_enable_rcc(spi_ports_t port) {
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
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  LL_SPI_TransmitData8(PORT.channel, data);

  for (uint16_t timeout = timeout_max; LL_SPI_IsActiveFlag_RXNE(PORT.channel) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  for (uint16_t timeout = timeout_max; LL_SPI_IsActiveFlag_BSY(PORT.channel) == SET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  return LL_SPI_ReceiveData8(PORT.channel);
}

volatile uint8_t dma_transfer_done[2 * 8] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#define DMA_TRANSFER_DONE dma_transfer_done[(PORT.dma.dma_port - 1) * 8 + PORT.dma.rx_stream_index]

void spi_dma_init(spi_ports_t port) {
  // Enable DMA clock
  spi_dma_enable_rcc(port);

  // 2bits for priority, 2bit for subpriority
  NVIC_SetPriorityGrouping(2);

  uint32_t priority = NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x02, 0x02);
  NVIC_SetPriority(PORT.dma.rx_it, priority);
  NVIC_EnableIRQ(PORT.dma.rx_it);

  DMA_TRANSFER_DONE = 1;
}

void spi_dma_receive_init(spi_ports_t port, uint8_t *base_address_in, uint32_t buffer_size) {
  //RX Stream
  LL_DMA_DeInit(PORT.dma.dma, PORT.dma.rx_stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.Channel = PORT.dma.channel;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)(&(PORT.channel->DR));
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

void spi_dma_transmit_init(spi_ports_t port, uint8_t *base_address_out, uint32_t buffer_size) {
  //TX Stream
  LL_DMA_DeInit(PORT.dma.dma, PORT.dma.tx_stream_index);

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
  return DMA_TRANSFER_DONE;
}

void spi_dma_wait_for_ready(spi_ports_t port) {
#ifdef BRUSHLESS_TARGET
  if (port == SPI_PORT1) {
    extern volatile int dshot_dma_phase;
    while (dshot_dma_phase != 0)
      ;
  }
#endif
  for (uint16_t timeout = 0x400; DMA_TRANSFER_DONE == 0; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return;
    }
    __WFI();
  }
}

void spi_dma_transfer_begin(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  spi_dma_wait_for_ready(port);
  DMA_TRANSFER_DONE = 0;

  spi_dma_receive_init(port, buffer, length);
  spi_dma_transmit_init(port, buffer, length);

  // Enable the SPI Rx/Tx DMA request
  LL_SPI_EnableDMAReq_TX(PORT.channel);
  LL_SPI_EnableDMAReq_RX(PORT.channel);

  // if everything goes right, those are not required.
  //DMA_ClearFlag(PORT.dma.rx_stream, PORT.dma.rx_tci_flag);
  //DMA_ClearFlag(PORT.dma.tx_stream, PORT.dma.tx_tci_flag);

  //DMA_ClearITPendingBit(PORT.dma.rx_stream, PORT.dma.rx_it_flag);
  LL_DMA_EnableIT_TC(PORT.dma.dma, PORT.dma.rx_stream_index);

  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
  LL_DMA_EnableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

  // now we can enable the peripheral
  //LL_SPI_Enable(PORT.channel);
}

//blocking dma transmit bytes
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  spi_dma_wait_for_ready(port);
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

/* could also be solved with the IT, but seems to be slower by 4us worstcase 
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  spi_dma_transfer_begin(port, buffer, length);
  spi_dma_wait_for_ready(port);
}
*/

static void handle_dma_rx_isr(spi_ports_t port) {
  if (dma_is_flag_active_tc(PORT.dma.dma, PORT.dma.rx_stream_index)) {
    dma_clear_flag_tc(PORT.dma.dma, PORT.dma.rx_stream_index);
    dma_clear_flag_tc(PORT.dma.dma, PORT.dma.tx_stream_index);

    LL_DMA_DisableIT_TC(PORT.dma.dma, PORT.dma.rx_stream_index);

    DMA_TRANSFER_DONE = 1;

    LL_SPI_DisableDMAReq_TX(PORT.channel);
    LL_SPI_DisableDMAReq_RX(PORT.channel);

    LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.rx_stream_index);
    LL_DMA_DisableStream(PORT.dma.dma, PORT.dma.tx_stream_index);

    // now we can disable the peripheral
    //LL_SPI_Disable(PORT.channel);

#if defined(ENABLE_OSD) && defined(MAX7456_SPI_PORT)
    if (port == MAX7456_SPI_PORT) {
      extern void max7456_dma_rx_isr();
      max7456_dma_rx_isr();
    }
#endif
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
