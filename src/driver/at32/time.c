#include "driver/time.h"

#include "core/project.h"

static volatile uint32_t systick_count = 0;
static volatile uint32_t systick_val = 0;
static volatile uint32_t systick_pending = 0;

static void debug_time_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void time_init() {
  system_clock_config();
  crm_periph_clock_enable(CRM_SCFG_PERIPH_CLOCK, TRUE);

  // interrupt only every 1ms
  systick_clock_source_config(SYSTICK_CLOCK_SOURCE_AHBCLK_NODIV);
  SysTick_Config(SystemCoreClock / 1000);

  debug_time_init();
}

void SysTick_Handler() {
  systick_count++;
  systick_val = SysTick->VAL;
  systick_pending = 0;
  (void)(SysTick->CTRL);
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
