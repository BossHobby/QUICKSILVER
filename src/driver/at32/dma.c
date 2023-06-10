#include "driver/dma.h"

#include "driver/rcc.h"

#define DMA_GL_FLAG ((uint32_t)0x00000001)
#define DMA_FDT_FLAG ((uint32_t)0x00000002)
#define DMA_HDT_FLAG ((uint32_t)0x00000004)
#define DMA_DTERR_FLAG ((uint32_t)0x00000008)

#define DMAMUX_DMAREQ_ID_TIM1_CH1 DMAMUX_DMAREQ_ID_TMR1_CH1
#define DMAMUX_DMAREQ_ID_TIM1_CH2 DMAMUX_DMAREQ_ID_TMR1_CH2
#define DMAMUX_DMAREQ_ID_TIM1_CH3 DMAMUX_DMAREQ_ID_TMR1_CH3
#define DMAMUX_DMAREQ_ID_TIM1_CH4 DMAMUX_DMAREQ_ID_TMR1_CH3

#define DMA_STREAMS          \
  DMA_STREAM(1, 1, SPI1_RX)  \
  DMA_STREAM(1, 2, SPI1_TX)  \
  DMA_STREAM(1, 3, SPI2_RX)  \
  DMA_STREAM(1, 4, SPI2_TX)  \
  DMA_STREAM(1, 5, SPI3_RX)  \
  DMA_STREAM(1, 6, SPI3_TX)  \
  DMA_STREAM(2, 1, SPI4_RX)  \
  DMA_STREAM(2, 2, SPI4_TX)  \
  DMA_STREAM(2, 3, TIM1_CH1) \
  DMA_STREAM(2, 4, TIM1_CH3) \
  DMA_STREAM(2, 5, TIM1_CH4)

#define DMA_STREAM(_port, _chan, _dev)           \
  [DMA_DEVICE_##_dev] = {                        \
      .device = DMA_DEVICE_##_dev,               \
      .port = DMA##_port,                        \
      .port_index = _port,                       \
      .channel = DMA##_port##_CHANNEL##_chan,    \
      .channel_index = _chan,                    \
      .request = DMAMUX_DMAREQ_ID_##_dev,        \
      .mux = DMA##_port##MUX_CHANNEL##_chan,     \
      .irq = DMA##_port##_Channel##_chan##_IRQn, \
  },

const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX] = {DMA_STREAMS};

#undef DMA_STREAM

void dma_prepare_tx_memory(void *addr, uint32_t size) {
}

void dma_prepare_rx_memory(void *addr, uint32_t size) {
}

void dma_enable_rcc(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  switch (dma->port_index) {
  case 1:
    rcc_enable(RCC_ENCODE(DMA1));
    dmamux_enable(DMA1, TRUE);
    break;
  case 2:
    rcc_enable(RCC_ENCODE(DMA2));
    dmamux_enable(DMA2, TRUE);
    break;
  }
}

static uint32_t dma_flag_for_channel(const dma_stream_def_t *dma, uint32_t val) {
  // 4bits per channel
  const uint32_t shift = (dma->channel_index - 1) * 4;
  const uint32_t port = dma->port_index == 2 ? 0x10000000 : 0x0;
  return port | (val << shift);
}

bool dma_is_flag_active_tc(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  const uint32_t flag = dma_flag_for_channel(dma, DMA_FDT_FLAG);
  return dma_flag_get(flag);
}

void dma_clear_flag_tc(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  dma_flag_clear(dma_flag_for_channel(dma, DMA_FDT_FLAG));
  dma_flag_clear(dma_flag_for_channel(dma, DMA_HDT_FLAG));
  dma_flag_clear(dma_flag_for_channel(dma, DMA_DTERR_FLAG));
}

extern void dshot_dma_isr(dma_device_t dev);
extern void spi_dma_isr(dma_device_t dev);

static void handle_dma_stream_isr(dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(dev);
    break;
  case DMA_DEVICE_TIM1_CH1:
  case DMA_DEVICE_TIM1_CH3:
  case DMA_DEVICE_TIM1_CH4:
#ifdef USE_MOTOR_DSHOT
    dshot_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_MAX:
    break;
  }
}

#define DMA_STREAM(_port, _chan, _dev)              \
  void DMA##_port##_Channel##_chan##_IRQHandler() { \
    handle_dma_stream_isr(DMA_DEVICE_##_dev);       \
  }

DMA_STREAMS

#undef DMA_STREAM