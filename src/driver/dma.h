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
  DMA_DEVICE(SERIAL_RX) \
  DMA_DEVICE(SERIAL_TX) \
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

#if defined(STM32F7) || defined(STM32H7)
#define DMA_CACHE_LINE_MASK (DMA_ALIGN_SIZE - 1)

// Inline, so MCUs without a data cache drop the calls and the others skip a
// function call for buffers in uncached memory.
inline bool dma_memory_cached(void *addr) {
  return !WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr);
}

// Every cache line the buffer touches, from its line-aligned start.
inline uint32_t dma_cache_line_start(void *addr) {
  return (uint32_t)addr & ~DMA_CACHE_LINE_MASK;
}
inline int32_t dma_cache_line_span(void *addr, uint32_t size) {
  const uint32_t end = ((uint32_t)addr + size + DMA_CACHE_LINE_MASK) & ~DMA_CACHE_LINE_MASK;
  return end - dma_cache_line_start(addr);
}

inline void dma_prepare_tx_memory(void *addr, uint32_t size) {
  if (dma_memory_cached(addr))
    SCB_CleanDCache_by_Addr((uint32_t *)dma_cache_line_start(addr), dma_cache_line_span(addr, size));
}
inline void dma_prepare_rx_memory(void *addr, uint32_t size) {
  if (dma_memory_cached(addr))
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)dma_cache_line_start(addr), dma_cache_line_span(addr, size));
}
#else
inline void dma_prepare_tx_memory(void *addr, uint32_t size) {}
inline void dma_prepare_rx_memory(void *addr, uint32_t size) {}
#endif

void dma_enable_rcc(const dma_stream_def_t *def);
dma_stream_t dma_claim_unused(dma_device_t device);
void dma_release(dma_stream_t stream, dma_device_t device);

#ifdef STM32F4
bool dma_can_use_dma2(dma_device_t device);
#endif
