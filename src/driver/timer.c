#include "driver/timer.h"

#include "driver/rcc.h"

static void timer_enable_rcc(TIM_TypeDef *tim) {
  switch ((uint32_t)tim) {
#ifdef TIM1
  case (uint32_t)TIM1:
    rcc_enable(RCC_APB2_GRP1(TIM1));
    break;
#endif
#ifdef TIM2
  case (uint32_t)TIM2:
    rcc_enable(RCC_APB1_GRP1(TIM2));
    break;
#endif
#ifdef TIM3
  case (uint32_t)TIM3:
    rcc_enable(RCC_APB1_GRP1(TIM3));
    break;
#endif
#ifdef TIM4
  case (uint32_t)TIM4:
    rcc_enable(RCC_APB1_GRP1(TIM4));
    break;
#endif
#ifdef TIM5
  case (uint32_t)TIM5:
    rcc_enable(RCC_APB1_GRP1(TIM5));
    break;
#endif
#ifdef TIM6
  case (uint32_t)TIM6:
    rcc_enable(RCC_APB1_GRP1(TIM6));
    break;
#endif
#ifdef TIM7
  case (uint32_t)TIM7:
    rcc_enable(RCC_APB1_GRP1(TIM7));
    break;
#endif
#ifdef TIM8
  case (uint32_t)TIM8:
    rcc_enable(RCC_APB2_GRP1(TIM8));
    break;
#endif
#ifdef TIM9
  case (uint32_t)TIM9:
    rcc_enable(RCC_APB2_GRP1(TIM9));
    break;
#endif
#ifdef TIM11
  case (uint32_t)TIM11:
    rcc_enable(RCC_APB2_GRP1(TIM11));
    break;
#endif
#ifdef TIM12
  case (uint32_t)TIM12:
    rcc_enable(RCC_APB1_GRP1(TIM12));
    break;
#endif
#ifdef TIM13
  case (uint32_t)TIM13:
    rcc_enable(RCC_APB1_GRP1(TIM13));
    break;
#endif
#ifdef TIM14
  case (uint32_t)TIM14:
    rcc_enable(RCC_APB1_GRP1(TIM14));
    break;
#endif
  }
}

void timer_init(TIM_TypeDef *tim, uint16_t divider, uint32_t period) {
  timer_enable_rcc(tim);

  LL_TIM_InitTypeDef tim_init;
  tim_init.Prescaler = divider - 1;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim_init.Autoreload = period;
  tim_init.ClockDivision = 0;
  tim_init.RepetitionCounter = 0;
  LL_TIM_Init(tim, &tim_init);
}