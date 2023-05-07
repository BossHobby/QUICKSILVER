#pragma once

#include "core/project.h"
#include "driver/rcc.h"
#include "driver/resource.h"

typedef enum {
  TIMER_INVALID,
  TIMER1,
  TIMER2,
  TIMER3,
  TIMER4,
  TIMER5,
#ifndef STM32F411
  TIMER6,
  TIMER7,
  TIMER8,
#endif
  TIMER9,
  TIMER10,
  TIMER11,
#ifndef STM32F411
  TIMER12,
  TIMER13,
  TIMER14,
#endif
#ifdef STM32H743
  TIMER15,
  TIMER16,
  TIMER17,
#endif
  TIMER_MAX,
} timer_index_t;

typedef enum {
  TIMER_CH_INVALID = 0,
  TIMER_CH1 = (0x1 << 0),
  TIMER_CH1N = (0x1 << 1),
  TIMER_CH2 = (0x1 << 2),
  TIMER_CH2N = (0x1 << 3),
  TIMER_CH3 = (0x1 << 4),
  TIMER_CH3N = (0x1 << 5),
  TIMER_CH4 = (0x1 << 6),
  TIMER_CH_ALL = 0xff,
} timer_channel_t;

typedef enum {
  TIMER_USE_FREE,
  TIMER_USE_MOTOR_DSHOT,
  TIMER_USE_MOTOR_PWM,
  TIMER_USE_ELRS,
  TIMER_USE_SOFT_SERIAL,
} timer_use_t;

typedef struct {
  rcc_reg_t rcc;
  IRQn_Type irq;
  TIM_TypeDef *instance;
} timer_def_t;

typedef struct {
  uint8_t use;
  resource_tag_t tag;
} timer_assigment_t;

extern const timer_def_t timer_defs[TIMER_MAX];

void timer_alloc_init();
bool timer_alloc_tag(timer_use_t use, resource_tag_t tag);
resource_tag_t timer_alloc(timer_use_t use);
uint32_t timer_ll_channel(timer_channel_t chan);

void timer_init(timer_index_t tim, LL_TIM_InitTypeDef *tim_init);
void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period);