#pragma once

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
