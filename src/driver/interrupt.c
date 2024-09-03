#include "driver/interrupt.h"

void interrupt_init() {
#ifndef SIMULATOR
  NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);
#endif
}

void interrupt_enable(IRQn_Type irq, uint32_t prio) {
#ifndef SIMULATOR
  NVIC_SetPriority(irq, prio);
  NVIC_EnableIRQ(irq);
#endif
}

void interrupt_disable(IRQn_Type irq) {
#ifndef SIMULATOR
  NVIC_DisableIRQ(irq);
#endif
}