#pragma once

#include <stdint.h>

#include "core/project.h"

#define DMA_ALLOC_BUFFER_SIZE 4096

void *dma_mem_alloc(uint32_t min_size);
void *dma_mem_realloc(void *ptr, uint32_t min_size);
void dma_mem_free(void *ptr);

void dma_prepare_tx_memory(void *addr, uint32_t size);
void dma_prepare_rx_memory(void *addr, uint32_t size);

void dma_enable_rcc(const dma_assigment_t *ass);
const dma_assigment_t *dma_alloc(resource_tag_t tag, dma_device_t dev);

bool dma_is_flag_active_tc(const dma_assigment_t *ass);
void dma_clear_flag_tc(const dma_assigment_t *ass);