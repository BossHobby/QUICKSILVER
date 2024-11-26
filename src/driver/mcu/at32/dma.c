#include "driver/dma.h"

#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#define DMA_STREAM(_port, _chan)                 \
  [DMA##_port##_STREAM##_chan] = {               \
      .port = DMA##_port,                        \
      .port_index = _port,                       \
      .stream = DMA##_port##_CHANNEL##_chan,     \
      .stream_index = _chan,                     \
      .mux = DMA##_port##MUX_CHANNEL##_chan,     \
      .irq = DMA##_port##_Channel##_chan##_IRQn, \
  },
const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {DMA_STREAMS};
#undef DMA_STREAM

void dma_enable_rcc(const dma_stream_def_t *def) {
  switch (def->port_index) {
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

void dma_prepare_tx_memory(void *addr, uint32_t size) {}
void dma_prepare_rx_memory(void *addr, uint32_t size) {}

extern void dshot_dma_isr(const dma_device_t);
extern void spi_dma_isr(const dma_device_t);
extern void rgb_dma_isr(const dma_device_t);

static void handle_dma_stream_isr(const dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_RX:
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

#define DMA_STREAM(_port, _stream) \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(dma_stream_map[DMA##_port##_STREAM##_stream]); }

DMA_STREAMS

#undef DMA_STREAM