#include "driver/dma.h"

// Stub DMA implementation for native builds

// Provide a minimal array to avoid linker errors
const dma_stream_def_t dma_stream_defs[1] = { 0 };

void dma_prepare_tx_memory(void *addr, uint32_t size) {
    // No-op in native builds
}

void dma_prepare_rx_memory(void *addr, uint32_t size) {
    // No-op in native builds
}

void dma_enable_rcc(const dma_stream_def_t *def) {
    // No-op in native builds
}