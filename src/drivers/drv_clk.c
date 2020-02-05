#include "binary.h"
#include "defines.h"
#include "project.h"

// the clock is already set, we turn pll off and set a new multiplier
void setclock(void) {
  // turn on HSI
  RCC->CR |= 1;
  // wait for HSI ready
  while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
  }
  // set clock source HSI
  RCC->CFGR &= (0xFFFFFFFC);
  // wait for switch
  while ((RCC->CFGR & B00001100)) {
  }
  // turn pll off
  RCC->CR &= (uint32_t)0xFEFFFFFF;

  while ((RCC->CR & RCC_CR_PLLRDY) != 0) {
  }

  //reset pll settings
  RCC->CFGR &= (uint32_t)0xFFC0FFFF;

#ifndef F405
#ifdef ENABLE_OVERCLOCK
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
  RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL16;
#else
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
  RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL12;
#endif
#endif

  //PLL on
  RCC->CR |= RCC_CR_PLLON;

  while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
  }
  //set PLL as source
  //	RCC->CFGR&=(0xFFFFFF|B11111110);
  RCC->CFGR |= (B00000010);

  //	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  //  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

  //wait until PLL is the clock
#ifdef ENABLE_OVERCLOCK
  // only if overclocked as it is redundant
  while ((RCC->CFGR & B00001100) != B00001000) {
  }
#endif
}

void clk_init(void) {
  // reset clock to default values
  // sets clock to 48Mhz

  // this executes anyway, it's in the .s file
  //	SystemInit();

  //  SetSysClock();

#ifdef ENABLE_OVERCLOCK
  // set clock to 64Mhz (PLL max multiplier)
  setclock();
#endif
}
