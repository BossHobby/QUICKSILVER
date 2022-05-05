#include "drv_dma.h"

#include <stdbool.h>

#include "project.h"

// DMA1 Stream0 SPI3_RX
// DMA1 Stream1
// DMA1 Stream2
// DMA1 Stream3 SPI2_RX
// DMA1 Stream4 SPI2_TX
// DMA1 Stream5
// DMA1 Stream6
// DMA1 Stream7 SPI3_TX

// DMA2 Stream0 SPI4_RX
// DMA2 Stream1 SPI4_TX
// DMA2 Stream2 SPI1_RX
// DMA2 Stream3 TIM1_CH1
// DMA2 Stream4 TIM1_CH4
// DMA2 Stream5 SPI1_TX
// DMA2 Stream6 TIM1_CH3
// DMA2 Stream7

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

void dma_prepare_tx_memory(uint8_t *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr)) {
    SCB_CleanDCache_by_Addr(
        (uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK),
        (((uint32_t)addr & CACHE_LINE_MASK) + size - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_prepare_rx_memory(uint8_t *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr)) {
    SCB_CleanInvalidateDCache_by_Addr(
        (uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK),
        (((uint32_t)addr & CACHE_LINE_MASK) + size - 1 + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}