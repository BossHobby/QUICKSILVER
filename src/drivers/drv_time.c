#include "drv_time.h"

#include "project.h"

void failloop(int val);

#ifdef STM32F4

void debug_time_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void time_init() {
  SystemCoreClockUpdate();
  debug_time_init();
}

uint32_t time_cycles() {
  return DWT->CYCCNT;
}

void time_delay_us(uint32_t us) {
  volatile uint32_t delay = us * (SystemCoreClock / 1000000L);
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

uint32_t time_micros() {
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

uint32_t time_millis() {
  static uint32_t total_millis = 0;
  static uint32_t last_millis = 0;

  const uint32_t millis = time_micros() / 1000;
  if (millis >= last_millis) {
    total_millis += millis - last_millis;
  } else {
    total_millis += ((UINT32_MAX / 1000) + millis) - last_millis;
  }

  last_millis = millis;
  return total_millis;
}

#endif

void time_delay_until(uint32_t uS) {
  while (time_micros() < uS) {
    __asm("NOP");
  }
}

void SysTick_Handler() {
}
