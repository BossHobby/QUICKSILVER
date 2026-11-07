#include "driver/time.h"

#include <FreeRTOS.h>

#include "core/project.h"

static volatile bool kernel_tick_active;
volatile uint32_t systick_count = 0;

// Whole microseconds since the cycle counter was zeroed, written only by
// SysTick_Handler and read by the inline time_micros() in time.h. See the
// STM32 implementation for why one word describes the timebase.
volatile uint32_t micros_base = 0;

// Run-time stats clock for the FreeRTOS kernel (FreeRTOSConfig.h).
extern "C" uint32_t rtos_runtime_clock(void) {
  return DWT->CYCCNT;
}

void time_init() {
  static_assert(configCPU_CLOCK_HZ == SYS_CLOCK_FREQ_HZ);

  DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // SysTick runs from the processor clock; interrupt only every 1ms.
  SysTick_Config(SYS_CLOCK_FREQ_HZ / 1000);
}

// Keep the existing running 1 ms timebase through kernel startup. Resetting
// SysTick here would make time_micros() jump.
extern "C" void vPortSetupTimerInterrupt() {
  static_assert(configTICK_RATE_HZ == 1000);
  NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
  kernel_tick_active = true;
}

// The kernel's own SysTick handler, renamed by script/freertos.py.
extern "C" void xPortSysTickHandler();

extern "C" void SysTick_Handler() {
  systick_count = systick_count + 1;
  micros_base = time_micros();
  if (kernel_tick_active) {
    xPortSysTickHandler();
  }
}

void time_delay_us(uint32_t us) {
  const uint32_t delay = US_TO_CYCLES(us);
  const uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __NOP();
  }
}

void time_delay_ms(uint32_t ms) {
  while (ms--)
    time_delay_us(1000);
}
