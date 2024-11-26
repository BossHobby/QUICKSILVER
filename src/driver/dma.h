#pragma once

#include <cbor.h>
#include <stdint.h>

#include "driver/mcu/system.h"
#include "driver/resource.h"

#define DMA_ALIGN_SIZE 32
#define DMA_ALIGN(offset) MEMORY_ALIGN(offset, DMA_ALIGN_SIZE)

#define DMA_DEVICES     \
  DMA_DEVICE(DSHOT_CH1) \
  DMA_DEVICE(DSHOT_CH2) \
  DMA_DEVICE(DSHOT_CH3) \
  DMA_DEVICE(SPI1_RX)   \
  DMA_DEVICE(SPI1_TX)   \
  DMA_DEVICE(SPI2_RX)   \
  DMA_DEVICE(SPI2_TX)   \
  DMA_DEVICE(SPI3_RX)   \
  DMA_DEVICE(SPI3_TX)   \
  DMA_DEVICE(SPI4_RX)   \
  DMA_DEVICE(SPI4_TX)   \
  DMA_DEVICE(RGB)

typedef enum {
  DMA_DEVICE_INVALID,
#define DMA_DEVICE(_name) DMA_DEVICE_##_name,
  DMA_DEVICES DMA_DEVICE_MAX
#undef DMA_DEVICE
} dma_device_t;

typedef enum {
  DMA_STREAM_INVALID,
#define DMA_STREAM(_port, _stream) DMA##_port##_STREAM##_stream,
  DMA_STREAMS DMA_STREAM_MAX
#undef DMA_STREAM
} dma_stream_t;

extern const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX];
extern dma_device_t dma_stream_map[DMA_STREAM_MAX];

cbor_result_t cbor_decode_dma_device_t(cbor_value_t *dec, dma_device_t *d);
cbor_result_t cbor_encode_dma_device_t(cbor_value_t *enc, const dma_device_t *d);

cbor_result_t cbor_decode_dma_stream_t(cbor_value_t *dec, dma_stream_t *d);
cbor_result_t cbor_encode_dma_stream_t(cbor_value_t *enc, const dma_stream_t *d);

void dma_prepare_tx_memory(void *addr, uint32_t size);
void dma_prepare_rx_memory(void *addr, uint32_t size);

void dma_enable_rcc(const dma_stream_def_t *def);
