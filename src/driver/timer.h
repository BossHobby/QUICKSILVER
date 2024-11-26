#pragma once

#include "core/project.h"
#include "driver/rcc.h"
#include "driver/resource.h"

#if defined(STM32G4)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(6)     \
  TIMER(7)     \
  TIMER(8)     \
  TIMER(15)    \
  TIMER(16)    \
  TIMER(17)    \
  TIMER(20)
#elif defined(STM32F411)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(9)     \
  TIMER(10)    \
  TIMER(11)
#elif defined(STM32F405)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(6)     \
  TIMER(7)     \
  TIMER(8)     \
  TIMER(9)     \
  TIMER(10)    \
  TIMER(11)    \
  TIMER(12)    \
  TIMER(13)    \
  TIMER(14)
#elif defined(STM32F7)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(6)     \
  TIMER(7)     \
  TIMER(8)     \
  TIMER(9)     \
  TIMER(10)    \
  TIMER(11)    \
  TIMER(12)    \
  TIMER(13)    \
  TIMER(14)
#elif defined(STM32H743)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(6)     \
  TIMER(7)     \
  TIMER(8)     \
  TIMER(12)    \
  TIMER(13)    \
  TIMER(14)    \
  TIMER(15)    \
  TIMER(16)    \
  TIMER(17)
#elif defined(AT32F4)
#define TIMERS \
  TIMER(1)     \
  TIMER(2)     \
  TIMER(3)     \
  TIMER(4)     \
  TIMER(5)     \
  TIMER(6)     \
  TIMER(7)     \
  TIMER(8)     \
  TIMER(9)     \
  TIMER(10)    \
  TIMER(11)    \
  TIMER(12)    \
  TIMER(13)    \
  TIMER(14)    \
  TIMER(20)
#endif

typedef enum {
  TIMER_INVALID,
#define TIMER(_num) TIMER##_num,
  TIMERS TIMER_MAX
#undef TIMER
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
  TIMER_CH4N = (0x1 << 7),
  TIMER_CH_ALL = 0xff,
} timer_channel_t;

typedef enum {
  TIMER_USE_FREE,
  TIMER_USE_MOTOR_DSHOT,
  TIMER_USE_MOTOR_PWM,
  TIMER_USE_RGB_LED,
  TIMER_USE_ELRS,
  TIMER_USE_SOFT_SERIAL,
} timer_use_t;

typedef struct {
  rcc_reg_t rcc;
  IRQn_Type irq;
  timer_dev_t *instance;
} timer_def_t;

typedef struct {
  uint8_t use;
  resource_tag_t tag;
} timer_assigment_t;

extern const timer_def_t timer_defs[TIMER_MAX];

void timer_alloc_init();
bool timer_alloc_tag(timer_use_t use, resource_tag_t tag);
resource_tag_t timer_alloc(timer_use_t use);
uint32_t timer_channel_val(timer_channel_t chan);
uint32_t timer_channel_addr(timer_dev_t *timer, timer_channel_t chan);
void timer_enable_dma_request(timer_index_t tim, timer_channel_t chan, bool state);

void timer_up_init(timer_index_t tim, uint16_t divider, uint32_t period);