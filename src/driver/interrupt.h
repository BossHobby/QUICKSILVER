#pragma once

#include "project.h"

#define MAX_PRIORITY 0x1
#define DMA_PRIORITY 0x2
#define EXTI_PRIORITY 0x3
#define UART_PRIORITY 0x4
#define TIMER_PRIORITY 0x5
#define USB_PRIORITY 0x6

static inline void __basepri_restore(uint8_t *val) {
  __set_BASEPRI(*val);
}

static inline uint8_t __basepri_set_max(uint8_t prio) {
  __set_BASEPRI_MAX(prio << (8U - __NVIC_PRIO_BITS));
  return 1;
}

// Run block with elevated BASEPRI (using BASEPRI_MAX), restoring BASEPRI on exit.
// All exit paths are handled. Implemented as for loop, does intercept break and continue
// Full memory barrier is placed at start and at exit of block
// __unused__ attribute is used to supress CLang warning
#define ATOMIC_BLOCK(prio) \
  for (uint8_t __basepri_save __attribute__((__cleanup__(__basepri_restore), __unused__)) = __get_BASEPRI(), __todo = __basepri_set_max(prio); __todo; __todo = 0)

__attribute__((always_inline)) static inline int __int_disable_irq() {
  volatile int primask = __get_PRIMASK();
  __disable_irq();
  return primask & 1;
}

__attribute__((always_inline)) static inline void __int_restore_irq(int *primask) {
  if ((*primask) == 0) {
    __enable_irq();
  }
}

#define ATOMIC_BLOCK_ALL \
  for (int __basepri_save __attribute__((__cleanup__(__int_restore_irq))) = __int_disable_irq(), __todo = 1; __todo; __todo = 0)

void interrupt_enable(IRQn_Type irq, uint32_t prio);
void interrupt_disable(IRQn_Type irq);