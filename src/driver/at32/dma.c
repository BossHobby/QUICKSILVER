#include "driver/dma.h"

#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#define DMA_STREAM_MAX 14

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

#define DMA_STREAM_INDEX(_port, _stream) ((_port - 1) * 7 + (_stream - 1))
#define DMA_STREAM(_port, _chan)                 \
  {                                              \
      .port = DMA##_port,                        \
      .port_index = _port,                       \
      .stream = DMA##_port##_CHANNEL##_chan,     \
      .stream_index = _chan,                     \
      .mux = DMA##_port##MUX_CHANNEL##_chan,     \
      .irq = DMA##_port##_Channel##_chan##_IRQn, \
  },
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

    for (uint32_t j = 0; j < DMA_STREAM_MAX; j++) {
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

void dma_enable_rcc(const dma_assigment_t *ass) {
  switch (ass->def->port_index) {
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

extern void dshot_dma_isr(const dma_assigment_t *);
extern void spi_dma_isr(const dma_assigment_t *);
extern void rgb_dma_isr(const dma_assigment_t *);

static void handle_dma_stream_isr(const dma_assigment_t *ass) {
  switch (ass->dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_RX:
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

#define DMA_STREAM(_port, _stream) \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(&dma_assigments[DMA_STREAM_INDEX(_port, _stream)]); }

DMA_STREAMS

#undef DMA_STREAM