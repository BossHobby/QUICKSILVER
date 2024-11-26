#pragma once

#include <stdint.h>

#ifdef STM32G4
#define DMA_FLAG_TE (0x1 << 3)
#define DMA_FLAG_HT (0x1 << 2)
#define DMA_FLAG_TC (0x1 << 1)
#define DMA_FLAG_GI (0x1 << 0)

#define dma_flag_for_channel(dev, flags) ((flags) << (dev->stream_index * 4))
#define dma_is_flag_active_tc(dev) READ_BIT(dev->port->ISR, dma_flag_for_channel(dev, DMA_FLAG_TC))
#define dma_clear_flag_tc(dev) WRITE_REG(dev->port->IFCR, dma_flag_for_channel(dev, DMA_FLAG_TC | DMA_FLAG_TE | DMA_FLAG_HT | DMA_FLAG_GI));

#define LL_DMA_EnableStream LL_DMA_EnableChannel
#define LL_DMA_DisableStream LL_DMA_DisableChannel
#define LL_DMA_IsEnabledStream LL_DMA_IsEnabledChannel

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

#else
#define DMA_FLAG_TC (0x1 << 5)
#define DMA_FLAG_HT (0x1 << 4)
#define DMA_FLAG_TE (0x1 << 3)
#define DMA_FLAG_DME (0x1 << 2)
#define DMA_FLAG_FE (0x1 << 0)

static const uint32_t _dma_flag_shift[] = {0, 6, 16, 22, 0, 6, 16, 22};
#define dma_flag_for_channel(dev, flags) ((flags) << _dma_flag_shift[dev->stream_index])

#define dma_is_flag_active_tc(dev) \
  READ_BIT((dev->stream_index < LL_DMA_STREAM_4) ? dev->port->LISR : dev->port->HISR, dma_flag_for_channel(dev, DMA_FLAG_TC))

#define dma_clear_flag_tc(dev)                                                                                       \
  {                                                                                                                  \
    if (dev->stream_index < LL_DMA_STREAM_4)                                                                         \
      WRITE_REG(dev->port->LIFCR, dma_flag_for_channel(dev, DMA_FLAG_TC | DMA_FLAG_TE | DMA_FLAG_HT | DMA_FLAG_FE)); \
    else                                                                                                             \
      WRITE_REG(dev->port->HIFCR, dma_flag_for_channel(dev, DMA_FLAG_TC | DMA_FLAG_TE | DMA_FLAG_HT | DMA_FLAG_FE)); \
  }

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

#endif

uint32_t dma_map_channel(uint32_t channel);