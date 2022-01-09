#pragma once

#include <stdint.h>

void dma_prepare_tx_memory(uint8_t *addr, uint32_t size);
void dma_prepare_rx_memory(uint8_t *addr, uint32_t size);