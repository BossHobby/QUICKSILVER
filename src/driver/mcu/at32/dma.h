#pragma once

#define DMA_GL_FLAG ((uint32_t)0x00000001)
#define DMA_FDT_FLAG ((uint32_t)0x00000002)
#define DMA_HDT_FLAG ((uint32_t)0x00000004)
#define DMA_DTERR_FLAG ((uint32_t)0x00000008)

// 4bits per stream
#define dma_flag_for_stream(dev, flags) ((dev->port_index == 2 ? 0x10000000 : 0x0) | ((flags) << ((dev->stream_index - 1) * 4)))

#define dma_is_flag_active_tc(dev) (dma_flag_get(dma_flag_for_stream(dev, DMA_FDT_FLAG)))
#define dma_clear_flag_tc(dev) dma_flag_clear(dma_flag_for_stream(dev, DMA_FDT_FLAG | DMA_HDT_FLAG | DMA_DTERR_FLAG))

#define DMA_STREAMS \
  DMA_STREAM(1, 1)  \
  DMA_STREAM(1, 2)  \
  DMA_STREAM(1, 3)  \
  DMA_STREAM(1, 4)  \
  DMA_STREAM(1, 5)  \
  DMA_STREAM(1, 6)  \
  DMA_STREAM(1, 7)  \
  DMA_STREAM(2, 1)  \
  DMA_STREAM(2, 2)  \
  DMA_STREAM(2, 3)  \
  DMA_STREAM(2, 4)  \
  DMA_STREAM(2, 5)  \
  DMA_STREAM(2, 6)  \
  DMA_STREAM(2, 7)
