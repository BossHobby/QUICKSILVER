#pragma once

#include <stm32f4xx_ll_tim.h>

void timer_init(TIM_TypeDef *tim, uint16_t divider, uint32_t period);