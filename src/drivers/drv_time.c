//
#include "drv_time.h"
#include "defines.h"
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

uint32_t debug_timer_millis() {
  return (debug_timer_micros()) / 1000;
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

void time_init() {
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
unsigned long time_update(void) {
  unsigned long maxticks = SysTick->LOAD;
  unsigned long ticks = SysTick->VAL;
  unsigned long quotient;
  unsigned long elapsedticks;
  static unsigned long remainder = 0; // carry forward the remainder ticks;

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

// return time in uS from start ( micros())
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

void SysTick_Handler(void) {
}

#endif

#ifdef F0
// divider by 8 is enabled in this systick config
static __INLINE uint32_t SysTick_Config2(uint32_t ticks) {
  if (ticks > SysTick_LOAD_RELOAD_Msk)
    return (1); /* Reload value impossible */

  SysTick->LOAD = (ticks & SysTick_LOAD_RELOAD_Msk) - 1;       /* set reload register */
  NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); /* set Priority for Cortex-M0 System Interrupts */
  SysTick->VAL = 0;                                            /* Load the SysTick Counter Value */
  SysTick->CTRL =                                              //SysTick_CTRL_CLKSOURCE_Msk |   // divide by 8
      SysTick_CTRL_TICKINT_Msk |
      SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
  return (0);                  /* Function successful */
}

void time_init() {

  if (SysTick_Config2(SYS_CLOCK_FREQ_HZ / 8)) // sets reload value       48,000,000/8 = 6000   64,000,000/8 = 8000
  {                                           // not able to set divider																				// determines how much time between interrupts
    failloop(5);
    while (1)
      ;
  }
}

// called at least once per 16ms or time will overflow
unsigned long time_update(void) {
  unsigned long maxticks = SysTick->LOAD;
  unsigned long ticks = SysTick->VAL;
  unsigned long quotient;
  unsigned long elapsedticks;
  static unsigned long remainder = 0; // carry forward the remainder ticks;

  if (ticks < lastticks) {
    elapsedticks = lastticks - ticks;
  } else {
    // overflow ( underflow really)
    elapsedticks = lastticks + (maxticks - ticks);
  }

  lastticks = ticks;
  elapsedticks += remainder;

#ifdef ENABLE_OVERCLOCK
  quotient = elapsedticks / 8;
  remainder = elapsedticks - quotient * 8;
#else
  // faster divide by 6, but requires that gettime
  // be called at minimum every 16ms
  // (max val for elapsedticks: 98303)
  quotient = elapsedticks * 43691 >> 18;
  remainder = elapsedticks - quotient * 6;
#endif
  globalticks = globalticks + quotient;

  return globalticks;
}

// return time in uS from start ( micros())
unsigned long gettime() {
  return time_update();
}
#ifdef ENABLE_OVERCLOCK
// delay in uS
void delay(uint32_t data) {
  volatile uint32_t count;
  count = data * 7;
  while (count--)
    ;
}
#else
// delay in uS
void delay(uint32_t data) {
  volatile uint32_t count;
  count = data * 5;
  while (count--)
    ;
}
#endif

void SysTick_Handler(void) {
}

void debug_timer_init() {
}

uint32_t debug_timer_micros() {
  return gettime();
}

uint32_t debug_timer_millis() {
  return (debug_timer_micros()) / 1000;
}

void debug_timer_delay_us(uint32_t us) {
  delay(us);
}
#endif

void delay_until(uint32_t uS) {
  while (gettime() < uS)
    ;
}