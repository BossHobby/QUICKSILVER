#pragma once

#include "project.h"

#define MAX_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x0, 0x0)
#define UART_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x0, 0x1)
#define TIMER_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x0, 0x2)
#define DMA_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x0)
#define EXTI_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x1, 0x1)
#define USB_PRIORITY NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x3, 0x3)

static inline void __basepri_restore(uint8_t *val) {
  __set_BASEPRI(*val);
}

static inline uint8_t __basepri_set_max(uint8_t prio) {
  __set_BASEPRI_MAX(prio);
  return 1;
}

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) \
  for (uint8_t __basepri_save __attribute__((__cleanup__(__basepri_restore), __unused__)) = __get_BASEPRI(), __todo = __basepri_set_max(prio); __todo; __todo = 0)

void interrupt_enable(IRQn_Type irq, uint32_t prio);
void interrupt_disable(IRQn_Type irq);