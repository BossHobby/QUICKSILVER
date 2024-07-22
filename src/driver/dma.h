#pragma once

#include <stdint.h>

#include "core/project.h"
#include "driver/resource.h"

#define DMA_ALIGN_SIZE 32
#define DMA_ALIGN(offset) MEMORY_ALIGN(offset, DMA_ALIGN_SIZE)

typedef enum {
  DMA_DEVICE_INVALID,

  DMA_DEVICE_DSHOT_CH1,
  DMA_DEVICE_DSHOT_CH2,
  DMA_DEVICE_DSHOT_CH3,

  DMA_DEVICE_SPI1_RX,
  DMA_DEVICE_SPI1_TX,
  DMA_DEVICE_SPI2_RX,
  DMA_DEVICE_SPI2_TX,
  DMA_DEVICE_SPI3_RX,
  DMA_DEVICE_SPI3_TX,
  DMA_DEVICE_SPI4_RX,
  DMA_DEVICE_SPI4_TX,

  DMA_DEVICE_RGB,

  DMA_DEVICE_MAX,
} dma_device_t;

typedef struct {
  dma_device_t dev;
  const dma_stream_def_t *def;
  const dma_channel_t *chan;
} dma_assigment_t;

const dma_assigment_t *dma_alloc(dma_device_t dev, resource_tag_t tag);

void dma_prepare_tx_memory(void *addr, uint32_t size);
void dma_prepare_rx_memory(void *addr, uint32_t size);

void dma_enable_rcc(const dma_assigment_t *ass);
