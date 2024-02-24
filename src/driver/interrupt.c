#include "driver/interrupt.h"

void interrupt_init() {
  NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUPING);
}

void interrupt_enable(IRQn_Type irq, uint32_t prio) {
  NVIC_SetPriority(irq, prio);
  NVIC_EnableIRQ(irq);
}

void interrupt_disable(IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
}