#include "drv_dma.h"

#include "project.h"

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