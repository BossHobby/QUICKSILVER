#include "driver/dma.h"

#include <stdbool.h>
#include <string.h>

#include "core/debug.h"
#include "core/failloop.h"
#include "driver/interrupt.h"
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

#define DMA_STREAMS             \
  DMA_STREAM(2, 3, 2, SPI1_RX)  \
  DMA_STREAM(2, 3, 5, SPI1_TX)  \
  DMA_STREAM(1, 0, 3, SPI2_RX)  \
  DMA_STREAM(1, 0, 4, SPI2_TX)  \
  DMA_STREAM(1, 0, 0, SPI3_RX)  \
  DMA_STREAM(1, 0, 7, SPI3_TX)  \
  DMA_STREAM(2, 4, 0, SPI4_RX)  \
  DMA_STREAM(2, 4, 1, SPI4_TX)  \
  DMA_STREAM(2, 6, 3, TIM1_CH1) \
  DMA_STREAM(2, 6, 6, TIM1_CH3) \
  DMA_STREAM(2, 6, 4, TIM1_CH4)

#ifdef STM32H7
#define DMA_STREAM(_port, _chan, _stream, _dev)   \
  [DMA_DEVICE_##_dev] = {                         \
      .device = DMA_DEVICE_##_dev,                \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .channel = -1,                              \
      .channel_index = -1,                        \
      .request = LL_DMAMUX1_REQ_##_dev,           \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#else
#define DMA_STREAM(_port, _chan, _stream, _dev)   \
  [DMA_DEVICE_##_dev] = {                         \
      .device = DMA_DEVICE_##_dev,                \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .channel = LL_DMA_CHANNEL_##_chan,          \
      .channel_index = _chan,                     \
      .request = -1,                              \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#endif

const dma_stream_def_t dma_stream_defs[DMA_DEVICE_MAX] = {DMA_STREAMS};

#undef DMA_STREAM

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

typedef struct _dma_allocation {
  uint32_t size;
  uint16_t magic;
  uint16_t free;

  struct _dma_allocation *prev;
  struct _dma_allocation *next;
} __attribute__((aligned(32))) dma_allocation_t;

#define DMA_ALIGN_SIZE 32
#define DMA_ALIGN(offset) ((offset + (DMA_ALIGN_SIZE - 1)) & -DMA_ALIGN_SIZE)

static DMA_RAM uint8_t buffer[DMA_ALLOC_BUFFER_SIZE];

void *dma_alloc(uint32_t min_size) {
  ATOMIC_BLOCK_ALL {
    dma_allocation_t *alloc = (dma_allocation_t *)buffer;
    while (alloc != NULL) {
      if (alloc->free && alloc->size >= min_size) {
        break;
      }
      if (alloc->next == NULL && alloc->size == 0) {
        alloc->size = DMA_ALIGN(min_size);
        alloc->next = NULL;
        alloc->prev = NULL;
        break;
      }
      if (alloc->next == NULL) {
        dma_allocation_t *new_alloc = (dma_allocation_t *)((void *)(alloc) + sizeof(dma_allocation_t) + alloc->size);
        if (((void *)(new_alloc) - (void *)(buffer)) >= DMA_ALLOC_BUFFER_SIZE) {
          failloop(FAILLOOP_FAULT);
        }

        new_alloc->size = DMA_ALIGN(min_size);
        new_alloc->next = NULL;
        new_alloc->prev = alloc;

        alloc->next = new_alloc;
        alloc = new_alloc;
        break;
      }
      alloc = alloc->next;
    }

    alloc->magic = 0xBEEF;
    alloc->free = 0;

    return (void *)(alloc) + sizeof(dma_allocation_t);
  }

  failloop(FAILLOOP_FAULT);
  return NULL;
}

void dma_free(void *ptr) {
  ATOMIC_BLOCK_ALL {
    dma_allocation_t *alloc = (dma_allocation_t *)(ptr - sizeof(dma_allocation_t));
    if (alloc->magic != 0xBEEF) {
      failloop(FAILLOOP_DMA);
    }

    alloc->free = 1;

    while (alloc->next != NULL) {
      alloc = alloc->next;
    }

    while (alloc != NULL && alloc->free) {
      alloc->free = 0;
      alloc->size = 0;
      alloc->next = NULL;

      dma_allocation_t *prev = alloc->prev;
      if (prev) {
        prev->next = NULL;
      }
      alloc->prev = NULL;

      alloc = prev;
    }
  }
}

void *dma_realloc(void *ptr, uint32_t min_size) {
  ATOMIC_BLOCK_ALL {
    dma_allocation_t *alloc = (dma_allocation_t *)(ptr - sizeof(dma_allocation_t));
    if (alloc->size >= min_size) {
      return ptr;
    }

    uint32_t old_size = alloc->size;
    dma_free(ptr);

    void *new_ptr = dma_alloc(min_size);
    if (new_ptr && new_ptr != ptr) {
      memcpy(new_ptr, ptr, old_size);
    }
    return new_ptr;
  }

  failloop(FAILLOOP_FAULT);
  return NULL;
}

void dma_prepare_tx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_prepare_rx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_enable_rcc(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  switch (dma->port_index) {
  case 1:
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    break;
  case 2:
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
    break;
  }
}

uint32_t dma_is_flag_active_tc(DMA_TypeDef *dma, uint32_t stream) {
  switch (stream) {
  case LL_DMA_STREAM_0:
    return LL_DMA_IsActiveFlag_TC0(dma);
  case LL_DMA_STREAM_1:
    return LL_DMA_IsActiveFlag_TC1(dma);
  case LL_DMA_STREAM_2:
    return LL_DMA_IsActiveFlag_TC2(dma);
  case LL_DMA_STREAM_3:
    return LL_DMA_IsActiveFlag_TC3(dma);
  case LL_DMA_STREAM_4:
    return LL_DMA_IsActiveFlag_TC4(dma);
  case LL_DMA_STREAM_5:
    return LL_DMA_IsActiveFlag_TC5(dma);
  case LL_DMA_STREAM_6:
    return LL_DMA_IsActiveFlag_TC6(dma);
  case LL_DMA_STREAM_7:
    return LL_DMA_IsActiveFlag_TC7(dma);
  }
  return 0;
}

uint32_t dma_is_flag_active_te(DMA_TypeDef *dma, uint32_t stream) {
  switch (stream) {
  case LL_DMA_STREAM_0:
    return LL_DMA_IsActiveFlag_TE0(dma);
  case LL_DMA_STREAM_1:
    return LL_DMA_IsActiveFlag_TE1(dma);
  case LL_DMA_STREAM_2:
    return LL_DMA_IsActiveFlag_TE2(dma);
  case LL_DMA_STREAM_3:
    return LL_DMA_IsActiveFlag_TE3(dma);
  case LL_DMA_STREAM_4:
    return LL_DMA_IsActiveFlag_TE4(dma);
  case LL_DMA_STREAM_5:
    return LL_DMA_IsActiveFlag_TE5(dma);
  case LL_DMA_STREAM_6:
    return LL_DMA_IsActiveFlag_TE6(dma);
  case LL_DMA_STREAM_7:
    return LL_DMA_IsActiveFlag_TE7(dma);
  }
  return 0;
}

void dma_clear_flag_tc(DMA_TypeDef *dma, uint32_t stream) {
  switch (stream) {
  case LL_DMA_STREAM_0:
    LL_DMA_ClearFlag_TC0(dma);
    LL_DMA_ClearFlag_TE0(dma);
    LL_DMA_ClearFlag_HT0(dma);
    LL_DMA_ClearFlag_FE0(dma);
    break;
  case LL_DMA_STREAM_1:
    LL_DMA_ClearFlag_TC1(dma);
    LL_DMA_ClearFlag_TE1(dma);
    LL_DMA_ClearFlag_HT1(dma);
    LL_DMA_ClearFlag_FE1(dma);
    break;
  case LL_DMA_STREAM_2:
    LL_DMA_ClearFlag_TC2(dma);
    LL_DMA_ClearFlag_TE2(dma);
    LL_DMA_ClearFlag_HT2(dma);
    LL_DMA_ClearFlag_FE2(dma);
    break;
  case LL_DMA_STREAM_3:
    LL_DMA_ClearFlag_TC3(dma);
    LL_DMA_ClearFlag_TE3(dma);
    LL_DMA_ClearFlag_HT3(dma);
    LL_DMA_ClearFlag_FE3(dma);
    break;
  case LL_DMA_STREAM_4:
    LL_DMA_ClearFlag_TC4(dma);
    LL_DMA_ClearFlag_TE4(dma);
    LL_DMA_ClearFlag_HT4(dma);
    LL_DMA_ClearFlag_FE4(dma);
    break;
  case LL_DMA_STREAM_5:
    LL_DMA_ClearFlag_TC5(dma);
    LL_DMA_ClearFlag_TE5(dma);
    LL_DMA_ClearFlag_HT5(dma);
    LL_DMA_ClearFlag_FE5(dma);
    break;
  case LL_DMA_STREAM_6:
    LL_DMA_ClearFlag_TC6(dma);
    LL_DMA_ClearFlag_TE6(dma);
    LL_DMA_ClearFlag_HT6(dma);
    LL_DMA_ClearFlag_FE6(dma);
    break;
  case LL_DMA_STREAM_7:
    LL_DMA_ClearFlag_TC7(dma);
    LL_DMA_ClearFlag_TE7(dma);
    LL_DMA_ClearFlag_HT7(dma);
    LL_DMA_ClearFlag_FE7(dma);
    break;
  }
}

extern void dshot_dma_isr(dma_device_t dev);
extern void spi_dma_isr(dma_device_t dev);

static void handle_dma_stream_isr(dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(dev);
    break;
  case DMA_DEVICE_TIM1_CH1:
  case DMA_DEVICE_TIM1_CH3:
  case DMA_DEVICE_TIM1_CH4:
#if defined(USE_DSHOT_DMA_DRIVER)
    dshot_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_MAX:
    break;
  }
}

#define DMA_STREAM(_port, _chan, _stream, _dev)      \
  void DMA##_port##_Stream##_stream##_IRQHandler() { \
    handle_dma_stream_isr(DMA_DEVICE_##_dev);        \
  }

DMA_STREAMS

#undef DMA_STREAM