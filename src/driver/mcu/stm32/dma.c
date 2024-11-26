#include "driver/dma.h"

#include "core/failloop.h"
#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

#if defined(STM32G4)
#define DMA_STREAM(_port, _stream)                 \
  [DMA##_port##_STREAM##_stream] = {               \
      .port = DMA##_port,                          \
      .port_index = _port,                         \
      .stream = DMA##_port##_Channel##_stream,     \
      .stream_index = LL_DMA_CHANNEL_##_stream,    \
      .irq = DMA##_port##_Channel##_stream##_IRQn, \
  },
#else
#define DMA_STREAM(_port, _stream)                \
  [DMA##_port##_STREAM##_stream] = {              \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#endif
const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {DMA_STREAMS};
#undef DMA_STREAM

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

void dma_enable_rcc(const dma_stream_def_t *def) {
#ifdef STM32G4
  rcc_enable(RCC_AHB1_GRP1(DMAMUX1));
#endif

  switch (def->port_index) {
  case 1:
    rcc_enable(RCC_AHB1_GRP1(DMA1));
    break;
  case 2:
    rcc_enable(RCC_AHB1_GRP1(DMA2));
    break;
  }
}

#if !defined(STM32H7) && !defined(STM32G4)
uint32_t dma_map_channel(uint32_t channel) {
  switch (channel) {
  case 0:
    return LL_DMA_CHANNEL_0;
  case 1:
    return LL_DMA_CHANNEL_1;
  case 2:
    return LL_DMA_CHANNEL_2;
  case 3:
    return LL_DMA_CHANNEL_3;
  case 4:
    return LL_DMA_CHANNEL_4;
  case 5:
    return LL_DMA_CHANNEL_5;
  case 6:
    return LL_DMA_CHANNEL_6;
  case 7:
    return LL_DMA_CHANNEL_7;
  default:
    failloop(FAILLOOP_DMA);
    return 0;
  }
}
#endif

extern void dshot_dma_isr(const dma_device_t);
extern void spi_dma_isr(const dma_device_t);
extern void rgb_dma_isr(const dma_device_t);

static void handle_dma_stream_isr(const dma_device_t dev) {
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
  case DMA_DEVICE_DSHOT_CH1:
  case DMA_DEVICE_DSHOT_CH2:
  case DMA_DEVICE_DSHOT_CH3:
#ifdef USE_MOTOR_DSHOT
    dshot_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
    rgb_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_INVALID:
  case DMA_DEVICE_MAX:
    break;
  }
}

#define DMA_STREAM(_port, _stream)                                                                                          \
  void DMA##_port##_Stream##_stream##_IRQHandler() { handle_dma_stream_isr(dma_stream_map[DMA##_port##_STREAM##_stream]); } \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(dma_stream_map[DMA##_port##_STREAM##_stream]); }

DMA_STREAMS

#undef DMA_STREAM