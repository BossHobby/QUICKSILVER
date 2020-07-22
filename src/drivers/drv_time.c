#include "drv_time.h"

#include "project.h"

void failloop(int val);

#ifdef STM32F4

void debug_timer_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// divider by 8 is enabled in this systick config
static __INLINE uint32_t SysTick_Config2(uint32_t ticks) {
  if (ticks > SysTick_LOAD_RELOAD_Msk)
    return 1;

  SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;
  NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
  SysTick->VAL = 0;

  // divide by 8 - sets systick clock to 21mhz
  SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

  return 0;
}

void timer_init() {
#ifdef STM32F4
  SystemCoreClockUpdate();

  if (SysTick_Config2(SystemCoreClock / 100)) {
    failloop(5);
    while (1)
      ;
  }
#endif

  debug_timer_init();
}

uint32_t timer_cycles() {
  return DWT->CYCCNT;
}

void timer_delay_us(uint32_t us) {
  volatile uint32_t delay = us * (SystemCoreClock / 1000000L);
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

uint32_t timer_micros() {
  static uint32_t total_micros = 0;
  static uint32_t last_micros = 0;

  const uint32_t micros = DWT->CYCCNT / (SystemCoreClock / 1000000L);
  if (micros >= last_micros) {
    total_micros += micros - last_micros;
  } else {
    total_micros += ((UINT32_MAX / (SystemCoreClock / 1000000L)) + micros) - last_micros;
  }

  last_micros = micros;
  return total_micros;
}

uint32_t timer_millis() {
  static uint32_t total_millis = 0;
  static uint32_t last_millis = 0;

  const uint32_t millis = timer_micros() / 1000;
  if (millis >= last_millis) {
    total_millis += millis - last_millis;
  } else {
    total_millis += ((UINT32_MAX / 1000) + millis) - last_millis;
  }

  last_millis = millis;
  return total_millis;
}

#endif

void timer_delay_until(uint32_t uS) {
  while (timer_micros() < uS) {
    __asm("NOP");
  }
}

void SysTick_Handler() {
}
