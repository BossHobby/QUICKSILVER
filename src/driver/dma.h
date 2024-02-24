#pragma once

#include <stdint.h>

#include "core/project.h"

#define DMA_ALIGN_SIZE 32
#define DMA_ALIGN(offset) MEMORY_ALIGN(offset, DMA_ALIGN_SIZE)

typedef enum {
  DMA_DEVICE_SPI1_RX,
  DMA_DEVICE_SPI1_TX,
  DMA_DEVICE_SPI2_RX,
  DMA_DEVICE_SPI2_TX,
  DMA_DEVICE_SPI3_RX,
  DMA_DEVICE_SPI3_TX,
  DMA_DEVICE_SPI4_RX,
  DMA_DEVICE_SPI4_TX,
  DMA_DEVICE_TIM1_CH1,
  DMA_DEVICE_TIM1_CH3,
  DMA_DEVICE_TIM1_CH4,

  DMA_DEVICE_MAX,
} dma_device_t;

extern const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX];

void dma_prepare_tx_memory(void *addr, uint32_t size);
void dma_prepare_rx_memory(void *addr, uint32_t size);

void dma_enable_rcc(dma_device_t dev);
