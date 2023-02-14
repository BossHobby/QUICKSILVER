#include "driver/time.h"

#include "project.h"

#if defined(STM32F7) || defined(STM32H7)
// See "RM CoreSight Architecture Specification"
// B2.3.10  "LSR and LAR, Software Lock Status Register and Software Lock Access Register"
// "E1.2.11  LAR, Lock Access Register"
#define DWT_LAR_UNLOCK_VALUE 0xC5ACCE55
#endif

static volatile uint32_t systick_count = 0;
static volatile uint32_t systick_val = 0;
static volatile uint32_t systick_pending = 0;

static void debug_time_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(STM32F7)
  DWT->LAR = DWT_LAR_UNLOCK_VALUE;
#endif
#if defined(STM32H7)
  ITM->LAR = DWT_LAR_UNLOCK_VALUE;
  DWT->LAR = DWT_LAR_UNLOCK_VALUE;
#endif
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void time_init() {
  SystemCoreClockUpdate();

#ifndef STM32H7
  __HAL_RCC_PWR_CLK_ENABLE();
#endif
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  // interrupt only every 1ms
  SysTick_Config(SystemCoreClock / 1000);

  debug_time_init();
}

void SysTick_Handler() {
  systick_count++;
  systick_val = SysTick->VAL;
  systick_pending = 0;
  (void)(SysTick->CTRL);

#ifdef USE_HAL_DRIVER
  // used by the HAL for some timekeeping and timeouts, should always be 1ms
  HAL_IncTick();
#endif
}

uint32_t time_micros_isr() {
  register uint32_t ticks = SysTick->VAL;

  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
    systick_pending = 1;
    ticks = SysTick->VAL;
  }

  register uint32_t count = systick_count;
  register uint32_t pending = systick_pending;

  return ((count + pending) * 1000) + (TICKS_PER_US * 1000 - ticks) / TICKS_PER_US;
}

uint32_t time_micros() {
  if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) || (__get_BASEPRI())) {
    return time_micros_isr();
  }

  register uint32_t count = 0;
  register uint32_t ticks = 0;

  do {
    // number of 1ms systicks
    count = systick_count;

    // number of ticks _between_ 1ms systicks
    ticks = SysTick->VAL;
  } while (count != systick_count || ticks > systick_val);

  return (count * 1000) + (TICKS_PER_US * 1000 - ticks) / TICKS_PER_US;
}

uint32_t time_millis() {
  return systick_count;
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

void time_delay_ms(uint32_t ms) {
  while (ms--)
    time_delay_us(1000);
}
