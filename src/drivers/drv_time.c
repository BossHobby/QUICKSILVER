#include "drv_time.h"

#include "project.h"

void failloop(int val);

unsigned long lastticks;
unsigned long globalticks;
volatile unsigned long systickcount = 0;

#ifndef SYS_CLOCK_FREQ_HZ
#define SYS_CLOCK_FREQ_HZ 48000000
#warning SYS_CLOCK_FREQ_HZ not present
#endif

#ifdef F4

void debug_timer_init() {
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t debug_timer_micros() {
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

void debug_timer_delay_us(uint32_t us) {
  volatile uint32_t delay = us * (SystemCoreClock / 1000000L);
  volatile uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < delay) {
    __asm("NOP");
  }
}

// divider by 8 is enabled in this systick config
static __INLINE uint32_t SysTick_Config2(uint32_t ticks) {
  if (ticks > SysTick_LOAD_RELOAD_Msk)
    return (1); /* Reload value impossible */

  SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;       /* set reload register */
  NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL = 0;                                            /* Load the SysTick Counter Value */
  SysTick->CTRL =                                              //SysTick_CTRL_CLKSOURCE_Msk |   						// divide by 8 - sets systick clock to 21mhz
      SysTick_CTRL_TICKINT_Msk |
      SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
  return (0);                  /* Function successful */
}

void timer_init() {
#ifdef F411
  if (SysTick_Config2(SystemCoreClock / 10)) { // not able to set divider
    failloop(5);
    while (1)
      ;
  }
#else
  if (SysTick_Config2(16000000)) { // not able to set divider
    failloop(5);
    while (1)
      ;
  }
#endif

  debug_timer_init();
}

// called at least once per 16ms or time will overflow
uint32_t time_update() {
  uint32_t maxticks = SysTick->LOAD;
  uint32_t ticks = SysTick->VAL;
  uint32_t quotient;
  uint32_t elapsedticks;
  static uint32_t remainder = 0; // carry forward the remainder ticks;

  if (ticks < lastticks) {
    elapsedticks = lastticks - ticks;
  } else {
    // overflow ( underflow really)
    elapsedticks = lastticks + (maxticks - ticks);
  }

  lastticks = ticks;
  elapsedticks += remainder;

#ifdef F411
  quotient = elapsedticks / 11;
  remainder = elapsedticks - quotient * 11;
#else
  quotient = elapsedticks / 21;
  remainder = elapsedticks - quotient * 21;
#endif

  globalticks = globalticks + quotient;

  return globalticks;
}

// return time in uS from start (micros())
uint32_t gettime() {
  return time_update();
}

// delay in uS
void delay(uint32_t data) {
#ifdef F411
  volatile uint32_t count = data * 12;
#else
  volatile uint32_t count = data * 28;
#endif
  while (count--)
    ;
}

uint32_t timer_micros() {
  return debug_timer_micros();
}

uint32_t timer_millis() {
  static uint32_t total_millis = 0;
  static uint32_t last_millis = 0;

  const uint32_t millis = debug_timer_micros() / 1000;
  if (millis >= last_millis) {
    total_millis += millis - last_millis;
  } else {
    total_millis += ((UINT32_MAX / 1000) + millis) - last_millis;
  }

  last_millis = millis;
  return total_millis;
}

void timer_delay_us(uint32_t us) {
  debug_timer_delay_us(us);
}

#endif


void SysTick_Handler(void) {
}

void delay_until(uint32_t uS) {
  while (gettime() < uS)
    ;
}