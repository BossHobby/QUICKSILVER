#include "driver/dma.h"

#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#define DMA_STREAM_MAX 16

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

#if defined(STM32G4)
#define DMA_STREAMS \
  DMA_STREAM(1, 1)  \
  DMA_STREAM(1, 2)  \
  DMA_STREAM(1, 3)  \
  DMA_STREAM(1, 4)  \
  DMA_STREAM(1, 5)  \
  DMA_STREAM(1, 6)  \
  DMA_STREAM(1, 7)  \
  DMA_STREAM(1, 8)  \
  DMA_STREAM(2, 1)  \
  DMA_STREAM(2, 2)  \
  DMA_STREAM(2, 3)  \
  DMA_STREAM(2, 4)  \
  DMA_STREAM(2, 5)  \
  DMA_STREAM(2, 6)  \
  DMA_STREAM(2, 7)  \
  DMA_STREAM(2, 8)
#define DMA_STREAM_INDEX(_port, _stream) ((_port - 1) * 8 + _stream - 1)
#define DMA_STREAM(_port, _stream)                 \
  {                                                \
      .port = DMA##_port,                          \
      .port_index = _port,                         \
      .stream = DMA##_port##_Channel##_stream,     \
      .stream_index = LL_DMA_CHANNEL_##_stream,    \
      .irq = DMA##_port##_Channel##_stream##_IRQn, \
  },
#else
#define DMA_STREAMS \
  DMA_STREAM(1, 0)  \
  DMA_STREAM(1, 1)  \
  DMA_STREAM(1, 2)  \
  DMA_STREAM(1, 3)  \
  DMA_STREAM(1, 4)  \
  DMA_STREAM(1, 5)  \
  DMA_STREAM(1, 6)  \
  DMA_STREAM(1, 7)  \
  DMA_STREAM(2, 0)  \
  DMA_STREAM(2, 1)  \
  DMA_STREAM(2, 2)  \
  DMA_STREAM(2, 3)  \
  DMA_STREAM(2, 4)  \
  DMA_STREAM(2, 5)  \
  DMA_STREAM(2, 6)  \
  DMA_STREAM(2, 7)
#define DMA_STREAM_INDEX(_port, _stream) ((_port - 1) * 8 + _stream)
#define DMA_STREAM(_port, _stream)                \
  {                                               \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#endif
static const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {DMA_STREAMS};
#undef DMA_STREAM

static const dma_channel_t dma_channels[] = {
#include "dma.in"
};
#define DMA_CHANNEL_MAX (sizeof(dma_channels) / sizeof(dma_channel_t))

static dma_assigment_t dma_assigments[DMA_STREAM_MAX] = {};

const dma_assigment_t *dma_alloc(dma_device_t dev, resource_tag_t tag) {
  for (uint32_t i = 0; i < DMA_CHANNEL_MAX; i++) {
    const dma_channel_t *chan = &dma_channels[i];
    if (chan->tag != tag) {
      continue;
    }

#if defined(STM32H7) || defined(STM32G4)
    const uint32_t start = 0;
    const uint32_t max = DMA_STREAM_MAX;
#else
    const uint32_t start = chan->port_index == -1 ? 0 : ((chan->port_index - 1) * 8 + chan->stream_index);
    const uint32_t max = chan->port_index == -1 ? DMA_STREAM_MAX : start + 1;
#endif
    for (uint32_t j = start; j < max; j++) {
      dma_assigment_t *ass = &dma_assigments[j];
      if (ass->dev != DMA_DEVICE_INVALID) {
        continue;
      }

      ass->dev = dev;
      ass->chan = chan;
      ass->def = &dma_stream_defs[j];
      return ass;
    }
  }

  return NULL;
}

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

void dma_enable_rcc(const dma_assigment_t *ass) {
#ifdef STM32G4
  rcc_enable(RCC_AHB1_GRP1(DMAMUX1));
#endif

  switch (ass->def->port_index) {
  case 1:
    rcc_enable(RCC_AHB1_GRP1(DMA1));
    break;
  case 2:
    rcc_enable(RCC_AHB1_GRP1(DMA2));
    break;
  }
}

extern void dshot_dma_isr(const dma_assigment_t *ass);
extern void spi_dma_isr(const dma_assigment_t *ass);
extern void rgb_dma_isr(const dma_assigment_t *ass);

static void handle_dma_stream_isr(const dma_assigment_t *ass) {
  switch (ass->dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(ass);
    break;
  case DMA_DEVICE_DSHOT_CH1:
  case DMA_DEVICE_DSHOT_CH2:
  case DMA_DEVICE_DSHOT_CH3:
#ifdef USE_MOTOR_DSHOT
    dshot_dma_isr(ass);
#endif
    break;
  case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
    rgb_dma_isr(ass);
#endif
    break;
  case DMA_DEVICE_INVALID:
  case DMA_DEVICE_MAX:
    break;
  }
}

#define DMA_STREAM(_port, _stream)                                                                                               \
  void DMA##_port##_Stream##_stream##_IRQHandler() { handle_dma_stream_isr(&dma_assigments[DMA_STREAM_INDEX(_port, _stream)]); } \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(&dma_assigments[DMA_STREAM_INDEX(_port, _stream)]); }

DMA_STREAMS

#undef DMA_STREAM