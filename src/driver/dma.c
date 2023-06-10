#include "driver/dma.h"

#include <string.h>

#include "core/failloop.h"
#include "driver/interrupt.h"
#include "util/util.h"

#define DMA_ALIGN_SIZE 32
#define DMA_ALIGN(offset) MEMORY_ALIGN(offset, DMA_ALIGN_SIZE)

typedef struct _dma_allocation {
  uint32_t size;
  uint16_t magic;
  uint16_t free;

  struct _dma_allocation *prev;
  struct _dma_allocation *next;
} __attribute__((aligned(DMA_ALIGN_SIZE))) dma_allocation_t;

static DMA_RAM uint8_t dma_buffer[DMA_ALLOC_BUFFER_SIZE];

void *dma_mem_alloc(uint32_t min_size) {
  ATOMIC_BLOCK_ALL {
    dma_allocation_t *alloc = (dma_allocation_t *)dma_buffer;
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
        if (((void *)(new_alloc) - (void *)(dma_buffer)) >= DMA_ALLOC_BUFFER_SIZE) {
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

void dma_mem_free(void *ptr) {
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

void *dma_mem_realloc(void *ptr, uint32_t min_size) {
  ATOMIC_BLOCK_ALL {
    dma_allocation_t *alloc = (dma_allocation_t *)(ptr - sizeof(dma_allocation_t));
    if (alloc->size >= min_size) {
      return ptr;
    }

    uint32_t old_size = alloc->size;
    dma_mem_free(ptr);

    void *new_ptr = dma_mem_alloc(min_size);
    if (new_ptr && new_ptr != ptr) {
      memcpy(new_ptr, ptr, old_size);
    }
    return new_ptr;
  }

  failloop(FAILLOOP_FAULT);
  return NULL;
}