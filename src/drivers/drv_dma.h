#pragma once

#include <stdint.h>

#include "project.h"

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

  DMA_TypeDef *port;
  uint8_t port_index;

  uint32_t channel;
  uint8_t channel_index;

  uint32_t request;

  DMA_Stream_TypeDef *stream;
  uint8_t stream_index;

  IRQn_Type irq;
} dma_stream_def_t;

extern const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX];

void dma_prepare_tx_memory(uint8_t *addr, uint32_t size);
void dma_prepare_rx_memory(uint8_t *addr, uint32_t size);

uint32_t dma_is_flag_active_tc(DMA_TypeDef *dma, uint32_t stream);
void dma_clear_flag_tc(DMA_TypeDef *dma, uint32_t stream);