#pragma once

#include "project.h"

#define UART_PRIORITY 0x0, 0x0
#define TIMER_PRIORITY 0x1, 0x0
#define EXTI_PRIORITY 0x1, 0x1
#define DMA_PRIORITY 0x2, 0x2

void interrupt_enable(IRQn_Type irq, uint32_t preempt_priority, uint32_t sub_priority);