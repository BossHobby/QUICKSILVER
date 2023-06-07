#pragma once

#include <stdint.h>

typedef enum {
  RCC_INVALID = 0,
  RCC_AHB1_GRP1,
  RCC_AHB2_GRP1,
#ifndef STM32F411
  RCC_AHB3_GRP1,
#endif
#ifdef STM32H7
  RCC_AHB4_GRP1,
#endif
  RCC_APB1_GRP1,
#ifdef STM32H7
  RCC_APB1_GRP2,
#endif
  RCC_APB2_GRP1,
#ifdef STM32H7
  RCC_APB3_GRP1,
  RCC_APB4_GRP1,
#endif
  RCC_MAX
} rcc_bus_t;

#define RCC_ENCODE(bus, periph) (rcc_reg_t)((((uint32_t)(bus)) << 8) | LOG2_32BIT(periph))

#define RCC_AHB1_GRP1(periph) RCC_ENCODE(RCC_AHB1_GRP1, LL_AHB1_GRP1_PERIPH_##periph)
#define RCC_AHB2_GRP1(periph) RCC_ENCODE(RCC_AHB2_GRP1, LL_AHB2_GRP1_PERIPH_##periph)
#define RCC_AHB3_GRP1(periph) RCC_ENCODE(RCC_AHB3_GRP1, LL_AHB3_GRP1_PERIPH_##periph)
#define RCC_AHB4_GRP1(periph) RCC_ENCODE(RCC_AHB4_GRP1, LL_AHB4_GRP1_PERIPH_##periph)
#define RCC_APB1_GRP1(periph) RCC_ENCODE(RCC_APB1_GRP1, LL_APB1_GRP1_PERIPH_##periph)
#define RCC_APB1_GRP2(periph) RCC_ENCODE(RCC_APB1_GRP2, LL_APB1_GRP2_PERIPH_##periph)
#define RCC_APB2_GRP1(periph) RCC_ENCODE(RCC_APB2_GRP1, LL_APB2_GRP1_PERIPH_##periph)
#define RCC_APB3_GRP1(periph) RCC_ENCODE(RCC_APB3_GRP1, LL_APB3_GRP1_PERIPH_##periph)
#define RCC_APB4_GRP1(periph) RCC_ENCODE(RCC_APB4_GRP1, LL_APB4_GRP1_PERIPH_##periph)