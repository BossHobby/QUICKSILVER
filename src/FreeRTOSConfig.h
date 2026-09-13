#pragma once

#include <stdint.h>

#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configUSE_TICKLESS_IDLE 0
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 2
// Bare idle has no local stack or FP context; allow for the saved integer
// context, alignment and overflow guard (128 bytes on Cortex-M).
// Recheck if idle hooks or task deletion cleanup are enabled.
#define configMINIMAL_STACK_SIZE 32
#define configMAX_TASK_NAME_LEN 8
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configKERNEL_PROVIDED_STATIC_MEMORY 1
#define configUSE_MUTEXES 0
#define configUSE_TASK_NOTIFICATIONS 0
#define configUSE_TIMERS 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_TRACE_FACILITY 0
#define configUSE_NEWLIB_REENTRANT 0

#ifndef SIMULATOR
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#ifdef __cplusplus
extern "C" {
#endif
#ifdef AT32
extern unsigned int system_core_clock;
#define configCPU_CLOCK_HZ system_core_clock
#else
extern uint32_t SystemCoreClock;
#define configCPU_CLOCK_HZ SystemCoreClock
#endif
#ifdef __cplusplus
}
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif
void thread_assert_failed(void);
#ifdef __cplusplus
}
#endif
#define configASSERT(condition) do { if (!(condition)) thread_assert_failed(); } while (0)
