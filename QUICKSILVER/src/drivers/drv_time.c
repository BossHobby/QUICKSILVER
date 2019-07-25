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

#ifdef F405
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
unsigned long gettime() {
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
#endif
