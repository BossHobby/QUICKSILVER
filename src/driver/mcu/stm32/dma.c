#include "driver/dma.h"

#include "driver/rcc.h"

#if !defined(STM32G4)
// DMA1 Stream0 SPI3_RX
// DMA1 Stream1
// DMA1 Stream2
// DMA1 Stream3 SPI2_RX
// DMA1 Stream4 SPI2_TX
// DMA1 Stream5
// DMA1 Stream6
// DMA1 Stream7 SPI3_TX

// DMA2 Stream0 SPI4_RX
// DMA2 Stream1 SPI4_TX
// DMA2 Stream2 SPI1_RX
// DMA2 Stream3 TIM1_CH1
// DMA2 Stream4 TIM1_CH4
// DMA2 Stream5 SPI1_TX
// DMA2 Stream6 TIM1_CH3
// DMA2 Stream7

#define DMA_STREAMS             \
  DMA_STREAM(2, 3, 2, SPI1_RX)  \
  DMA_STREAM(2, 3, 5, SPI1_TX)  \
  DMA_STREAM(1, 0, 3, SPI2_RX)  \
  DMA_STREAM(1, 0, 4, SPI2_TX)  \
  DMA_STREAM(1, 0, 0, SPI3_RX)  \
  DMA_STREAM(1, 0, 7, SPI3_TX)  \
  DMA_STREAM(2, 4, 0, SPI4_RX)  \
  DMA_STREAM(2, 4, 1, SPI4_TX)  \
  DMA_STREAM(2, 6, 3, TIM1_CH1) \
  DMA_STREAM(2, 6, 6, TIM1_CH3) \
  DMA_STREAM(2, 6, 4, TIM1_CH4)
#else
#define DMA_STREAMS             \
  DMA_STREAM(1, 0, 1, SPI1_RX)  \
  DMA_STREAM(1, 0, 2, SPI1_TX)  \
  DMA_STREAM(1, 0, 3, SPI2_RX)  \
  DMA_STREAM(1, 0, 4, SPI2_TX)  \
  DMA_STREAM(1, 0, 5, SPI3_RX)  \
  DMA_STREAM(1, 0, 6, SPI3_TX)  \
  DMA_STREAM(1, 0, 7, SPI4_RX)  \
  DMA_STREAM(1, 0, 8, SPI4_TX)  \
  DMA_STREAM(2, 0, 1, TIM1_CH1) \
  DMA_STREAM(2, 0, 2, TIM1_CH3) \
  DMA_STREAM(2, 0, 3, TIM1_CH4)
#endif

#if defined(STM32G4)
#define DMA_STREAM(_port, _chan, _stream, _dev)    \
  [DMA_DEVICE_##_dev] = {                          \
      .device = DMA_DEVICE_##_dev,                 \
      .port = DMA##_port,                          \
      .port_index = _port,                         \
      .channel = 0,                                \
      .channel_index = 0,                          \
      .request = LL_DMAMUX_REQ_##_dev,             \
      .stream = DMA##_port##_Channel##_stream,     \
      .stream_index = LL_DMA_CHANNEL_##_stream,    \
      .irq = DMA##_port##_Channel##_stream##_IRQn, \
  },
#elif defined(STM32H7)
#define DMA_STREAM(_port, _chan, _stream, _dev)   \
  [DMA_DEVICE_##_dev] = {                         \
      .device = DMA_DEVICE_##_dev,                \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .channel = -1,                              \
      .channel_index = -1,                        \
      .request = LL_DMAMUX1_REQ_##_dev,           \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#else
#define DMA_STREAM(_port, _chan, _stream, _dev)   \
  [DMA_DEVICE_##_dev] = {                         \
      .device = DMA_DEVICE_##_dev,                \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .channel = LL_DMA_CHANNEL_##_chan,          \
      .channel_index = _chan,                     \
      .request = -1,                              \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#endif

const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX] = {DMA_STREAMS};

#undef DMA_STREAM

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

void dma_prepare_tx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_prepare_rx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_enable_rcc(dma_device_t dev) {
#ifdef STM32G4
  rcc_enable(RCC_AHB1_GRP1(DMAMUX1));
#endif

  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  switch (dma->port_index) {
  case 1:
    rcc_enable(RCC_AHB1_GRP1(DMA1));
    break;
  case 2:
    rcc_enable(RCC_AHB1_GRP1(DMA2));
    break;
  }
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

#define DMA_STREAM(_port, _chan, _stream, _dev)                                                  \
  void DMA##_port##_Stream##_stream##_IRQHandler() { handle_dma_stream_isr(DMA_DEVICE_##_dev); } \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(DMA_DEVICE_##_dev); }

DMA_STREAMS

#undef DMA_STREAM