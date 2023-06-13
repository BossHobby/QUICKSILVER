#pragma once

#include "hardware.h"

#include "driver/resource.h"

typedef enum {
  DMA_DEVICE_INVALID,

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
  DMA_DEVICE_RGB,

  DMA_DEVICE_MAX,
} dma_device_t;

typedef struct {
  DMA_TypeDef *port;
  uint8_t port_index;

  DMA_Stream_TypeDef *stream;
  uint8_t stream_index;

  IRQn_Type irq;
} dma_stream_def_t;

typedef struct {
  dma_device_t dev;
  const dma_stream_def_t *def;
  uint32_t channel;
} dma_assigment_t;