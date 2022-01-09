#include "drv_interrupt.h"

void interrupt_enable(IRQn_Type irq, uint32_t preempt_priority, uint32_t sub_priority) {
  NVIC_SetPriorityGrouping(2);
  NVIC_SetPriority(irq, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), preempt_priority, sub_priority));
  NVIC_EnableIRQ(irq);
}

void interrupt_disable(IRQn_Type irq) {
  NVIC_DisableIRQ(irq);
}