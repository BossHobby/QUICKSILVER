#include "driver/dma.h"

#include "driver/interrupt.h"

#define DMA_STREAM(_port, _chan) _chan,
const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {0, DMA_STREAMS};
#undef DMA_STREAM

extern void spi_dma_isr(const dma_device_t);
extern void rgb_dma_isr(const dma_device_t);

// Every driver routes its channels' completion interrupts to DMA_IRQ_0.
extern "C" void DMA_IRQ_0_Handler() {
  uint32_t pending = dma_hw->ints0;
  dma_hw->ints0 = pending;

  while (pending) {
    const uint32_t channel = __builtin_ctz(pending);
    pending &= pending - 1;

    const dma_device_t dev = dma_stream_map[channel + 1];
    switch (dev) {
    case DMA_DEVICE_SPI1_RX:
    case DMA_DEVICE_SPI2_RX:
      spi_dma_isr(dev);
      break;
    case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
      rgb_dma_isr(dev);
#endif
      break;
    default:
      break;
    }
  }
}
