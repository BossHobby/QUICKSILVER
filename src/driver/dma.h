#pragma once

#include <stdint.h>

#include "core/project.h"

#define DMA_ALLOC_BUFFER_SIZE 4096

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

typedef struct {
  dma_device_t device;

  dma_port_t *port;
  uint8_t port_index;

  uint32_t channel;
  uint8_t channel_index;

  uint32_t request;

  dma_stream_t *stream;
  uint8_t stream_index;

  IRQn_Type irq;
} dma_stream_def_t;

extern const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX];

void *dma_alloc(uint32_t min_size);
void *dma_realloc(void *ptr, uint32_t min_size);
void dma_free(void *ptr);

void dma_prepare_tx_memory(void *addr, uint32_t size);
void dma_prepare_rx_memory(void *addr, uint32_t size);

void dma_enable_rcc(dma_device_t dev);

uint32_t dma_is_flag_active_tc(dma_port_t *dma, uint32_t stream);
uint32_t dma_is_flag_active_te(dma_port_t *dma, uint32_t stream);

void dma_clear_flag_tc(dma_port_t *dma, uint32_t stream);