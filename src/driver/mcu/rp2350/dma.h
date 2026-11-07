#pragma once

// The 16 channels; any channel serves any request, so targets may leave
// them unassigned and drivers claim one with dma_claim_unused().
#define DMA_STREAMS \
  DMA_STREAM(1, 0)  \
  DMA_STREAM(1, 1)  \
  DMA_STREAM(1, 2)  \
  DMA_STREAM(1, 3)  \
  DMA_STREAM(1, 4)  \
  DMA_STREAM(1, 5)  \
  DMA_STREAM(1, 6)  \
  DMA_STREAM(1, 7)  \
  DMA_STREAM(1, 8)  \
  DMA_STREAM(1, 9)  \
  DMA_STREAM(1, 10) \
  DMA_STREAM(1, 11) \
  DMA_STREAM(1, 12) \
  DMA_STREAM(1, 13) \
  DMA_STREAM(1, 14) \
  DMA_STREAM(1, 15)

#define dma_stream_channel(stream) (dma_stream_defs[stream])
