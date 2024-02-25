#pragma once

#define DMA_GL_FLAG ((uint32_t)0x00000001)
#define DMA_FDT_FLAG ((uint32_t)0x00000002)
#define DMA_HDT_FLAG ((uint32_t)0x00000004)
#define DMA_DTERR_FLAG ((uint32_t)0x00000008)

// 4bits per channel
#define dma_flag_for_channel(dev, flags) ((dev->port_index == 2 ? 0x10000000 : 0x0) | ((flags) << ((dev->channel_index - 1) * 4)))

#define dma_is_flag_active_tc(dev) (dma_flag_get(dma_flag_for_channel(dev, DMA_FDT_FLAG)))
#define dma_clear_flag_tc(dev) dma_flag_clear(dma_flag_for_channel(dev, DMA_FDT_FLAG | DMA_HDT_FLAG | DMA_DTERR_FLAG))
