#include "driver/time.h"

#include <FreeRTOS.h>

#include "core/project.h"

static volatile bool kernel_tick_active;
volatile uint32_t systick_count = 0;

// Whole microseconds since the cycle counter was zeroed, written only by
// SysTick_Handler and read by the inline time_micros() in time.h. Advancing it
// in whole microseconds keeps its cycle anchor at micros_base * TICKS_PER_US
// (mod 2^32), so one word describes the pair and readers in any context see
// either the old or the new value. Any published
// value stays valid while the counter has advanced less than 2^32 cycles past
// it; the 1 ms tick keeps that far below the wraparound.
volatile uint32_t micros_base = 0;

// Run-time stats clock for the FreeRTOS kernel (FreeRTOSConfig.h).
extern "C" uint32_t rtos_runtime_clock(void) {
  return DWT->CYCCNT;
}

static void debug_time_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void time_init() {
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

  debug_time_init();

  // interrupt only every 1ms
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  SysTick_Config(SYS_CLOCK_FREQ_HZ / 1000);
}

// Keep the existing running 1 ms timebase through kernel startup. Resetting
// SysTick here would make time_micros() jump and lose HAL time.
extern "C" void vPortSetupTimerInterrupt() {
  static_assert(configTICK_RATE_HZ == 1000);
  NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
  kernel_tick_active = true;
}

extern "C" void xPortSysTickHandler();

extern "C" void SysTick_Handler() {
  systick_count = systick_count + 1;
  micros_base = time_micros();
  if (kernel_tick_active) {
    xPortSysTickHandler();
  }
}

void time_delay_us(uint32_t us) {
  volatile uint32_t delay = us * (SYS_CLOCK_FREQ_HZ / 1000000L);
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

void time_delay_ms(uint32_t ms) {
  while (ms--)
    time_delay_us(1000);
}
