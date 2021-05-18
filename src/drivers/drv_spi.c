#include "drv_spi.h"

#include "usb_configurator.h"

#define SPI_DMA(spi_prt, dma_prt, chan, rx, tx) \
  {                                             \
    .dma_port = dma_prt,                        \
    .channel = DMA_Channel_##chan,              \
    .channel_index = chan,                      \
                                                \
    .rx_stream_index = rx,                      \
    .rx_stream = DMA##dma_prt##_Stream##rx,     \
    .rx_tci_flag = DMA_FLAG_TCIF##rx,           \
    .rx_it = DMA##dma_prt##_Stream##rx##_IRQn,  \
    .rx_it_flag = DMA_IT_TCIF##rx,              \
                                                \
    .tx_stream_index = tx,                      \
    .tx_stream = DMA##dma_prt##_Stream##tx,     \
    .tx_tci_flag = DMA_FLAG_TCIF##tx,           \
    .tx_it = DMA##dma_prt##_Stream##tx##_IRQn,  \
    .tx_it_flag = DMA_IT_TCIF##tx,              \
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

void spi_enable_rcc(spi_ports_t port) {
  switch (PORT.channel_index) {
  case 1:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    break;
  case 2:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    break;
  case 3:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    break;
  }
}

void spi_dma_enable_rcc(spi_ports_t port) {
  switch (PORT.dma.dma_port) {
  case 1:
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    break;
  case 2:
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
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
  GPIO_InitTypeDef gpio_init;

  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_pin_init_af(&gpio_init, PORT.sck, PORT.gpio_af);

  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_pin_init_af(&gpio_init, PORT.miso, PORT.gpio_af);

  gpio_init.GPIO_Mode = GPIO_Mode_AF;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_pin_init_af(&gpio_init, PORT.mosi, PORT.gpio_af);

  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType = GPIO_OType_PP;
  gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
  gpio_pin_init(&gpio_init, nss);
  gpio_pin_set(nss);
}

uint8_t spi_transfer_byte(spi_ports_t port, uint8_t data) {
  return spi_transfer_byte_timeout(port, data, 0x400);
}

uint8_t spi_transfer_byte_timeout(spi_ports_t port, uint8_t data, uint32_t timeout_max) {
  for (uint16_t timeout = timeout_max; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  SPI_I2S_SendData(PORT.channel, data);

  for (uint16_t timeout = timeout_max; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_RXNE) == RESET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  for (uint16_t timeout = timeout_max; SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_BSY) == SET; timeout--) {
    if (timeout == 0) {
      //liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
      liberror++;
      return 0;
    }
  }

  return SPI_I2S_ReceiveData(PORT.channel);
}

volatile uint8_t dma_transfer_done[2 * 8] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#define DMA_TRANSFER_DONE dma_transfer_done[(PORT.dma.dma_port - 1) * 8 + PORT.dma.rx_stream_index]

void spi_dma_init(spi_ports_t port) {
  // Enable DMA clock
  spi_dma_enable_rcc(port);

  // Enable DMA Interrupt on receive line
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = PORT.dma.rx_it;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStruct);

  DMA_TRANSFER_DONE = 1;
}

void spi_dma_receive_init(spi_ports_t port, uint8_t *base_address_in, uint32_t buffer_size) {
  //RX Stream
  DMA_DeInit(PORT.dma.rx_stream);

  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = PORT.dma.channel;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(PORT.channel->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_in;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(PORT.dma.rx_stream, &DMA_InitStructure);
}

void spi_dma_transmit_init(spi_ports_t port, uint8_t *base_address_out, uint32_t buffer_size) {
  //TX Stream
  DMA_DeInit(PORT.dma.tx_stream);

  DMA_InitTypeDef DMA_InitStructure;
  DMA_InitStructure.DMA_Channel = PORT.dma.channel;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(PORT.channel->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_out;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(PORT.dma.tx_stream, &DMA_InitStructure);
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
  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Rx, ENABLE);

  // if everything goes right, those are not required.
  //DMA_ClearFlag(PORT.dma.rx_stream, PORT.dma.rx_tci_flag);
  //DMA_ClearFlag(PORT.dma.tx_stream, PORT.dma.tx_tci_flag);

  //DMA_ClearITPendingBit(PORT.dma.rx_stream, PORT.dma.rx_it_flag);
  DMA_ITConfig(PORT.dma.rx_stream, DMA_IT_TC, ENABLE);

  DMA_Cmd(PORT.dma.rx_stream, ENABLE); // Enable the DMA SPI RX Stream
  DMA_Cmd(PORT.dma.tx_stream, ENABLE); // Enable the DMA SPI TX Stream

  // now we can enable the peripheral
  //SPI_Cmd(PORT.channel, ENABLE);
}

//blocking dma transmit bytes
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  spi_dma_wait_for_ready(port);
  DMA_TRANSFER_DONE = 0;

  spi_dma_receive_init(port, buffer, length);
  spi_dma_transmit_init(port, buffer, length);

  // Enable the SPI Rx/Tx DMA request
  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Rx, ENABLE);

  DMA_Cmd(PORT.dma.rx_stream, ENABLE); // Enable the DMA SPI RX Stream
  DMA_Cmd(PORT.dma.tx_stream, ENABLE); // Enable the DMA SPI TX Stream

  while (DMA_GetFlagStatus(PORT.dma.rx_stream, PORT.dma.rx_tci_flag) == RESET)
    ;
  while (DMA_GetFlagStatus(PORT.dma.tx_stream, PORT.dma.tx_tci_flag) == RESET)
    ;

  DMA_ClearFlag(PORT.dma.rx_stream, PORT.dma.rx_tci_flag);
  DMA_ClearFlag(PORT.dma.tx_stream, PORT.dma.tx_tci_flag);

  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Rx, DISABLE);

  DMA_Cmd(PORT.dma.rx_stream, DISABLE);
  DMA_Cmd(PORT.dma.tx_stream, DISABLE);

  DMA_TRANSFER_DONE = 1;
}

/* could also be solved with the IT, but seems to be slower by 4us worstcase 
void spi_dma_transfer_bytes(spi_ports_t port, uint8_t *buffer, uint32_t length) {
  spi_dma_transfer_begin(port, buffer, length);
  spi_dma_wait_for_ready(port);
}
*/

static void handle_dma_rx_isr(spi_ports_t port) {
  if (DMA_GetITStatus(PORT.dma.rx_stream, PORT.dma.rx_it_flag)) {
    DMA_ClearITPendingBit(PORT.dma.rx_stream, PORT.dma.rx_it_flag);
    DMA_ITConfig(PORT.dma.rx_stream, DMA_IT_TC, DISABLE);

    DMA_TRANSFER_DONE = 1;

    DMA_ClearFlag(PORT.dma.rx_stream, PORT.dma.rx_tci_flag);
    DMA_ClearFlag(PORT.dma.tx_stream, PORT.dma.tx_tci_flag);

    SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI_I2S_DMACmd(PORT.channel, SPI_I2S_DMAReq_Rx, DISABLE);

    DMA_Cmd(PORT.dma.rx_stream, DISABLE);
    DMA_Cmd(PORT.dma.tx_stream, DISABLE);

    // now we can disable the peripheral
    //SPI_Cmd(PORT.channel, DISABLE);

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
