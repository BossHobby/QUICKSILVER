#include "driver/rcc.h"

void rcc_enable(rcc_reg_t reg) {
  const rcc_bus_t bus = reg >> 8;
  const uint32_t periph = 0x1 << (reg & 0xFF);

  switch (bus) {
  case RCC_AHB1_GRP1:
    LL_AHB1_GRP1_EnableClock(periph);
    break;
  case RCC_AHB2_GRP1:
    LL_AHB2_GRP1_EnableClock(periph);
    break;
#ifndef STM32F411
  case RCC_AHB3_GRP1:
    LL_AHB3_GRP1_EnableClock(periph);
    break;
#endif
#ifdef STM32H7
  case RCC_AHB4_GRP1:
    LL_AHB4_GRP1_EnableClock(periph);
    break;
#endif
  case RCC_APB1_GRP1:
    LL_APB1_GRP1_EnableClock(periph);
    break;
#ifdef STM32H7
  case RCC_APB1_GRP2:
    LL_APB1_GRP2_EnableClock(periph);
    break;
#endif
  case RCC_APB2_GRP1:
    LL_APB2_GRP1_EnableClock(periph);
    break;
#ifdef STM32H7
  case RCC_APB3_GRP1:
    LL_APB3_GRP1_EnableClock(periph);
    break;
  case RCC_APB4_GRP1:
    LL_APB4_GRP1_EnableClock(periph);
    break;
#endif
  default:
    break;
  }
}