#include <math.h>

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_tim.h>

#include "control.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"

#ifdef USE_PWM_DRIVER

#ifndef PWM_CLOCK_FREQ_HZ
#define PWM_CLOCK_FREQ_HZ 48000000
#warning PWM_CLOCK_FREQ_HZ not present
#endif

#define PWM_DIVIDER 1
#define PWMTOP ((PWM_CLOCK_FREQ_HZ / PWMFREQ) - 1)

// pwm frequency checking macros
#if (PWMTOP < 1400)
// approx 34Khz
#undef PWMTOP
#define PWMTOP 6000
#warning PWM FREQUENCY TOO HIGH
#endif

#if (PWMTOP > 65535)
// under approx 732Hz we add the divider by 4
#undef PWMTOP
#define PWMTOP (((PWM_CLOCK_FREQ_HZ / 4) / PWMFREQ) - 1)
#undef PWM_DIVIDER
#define PWM_DIVIDER 4
//#warning PWM DIVIDE BY 4 ON
#endif

#if (PWMTOP > 65535)
// approx 183Hz is min frequency
#undef PWMTOP
#undef PWM_DIVIDER
#define PWMTOP 6000
#define PWM_DIVIDER 1
#warning PWM FREQUENCY TOO LOW
#endif
// end pwm frequency macros

extern profile_t profile;

unsigned long motorbeeptime = 0;
int beepon = 0;

// default value if not defined elsewhere
#ifndef MOTOR_BEEPS_TIMEOUT
#define MOTOR_BEEPS_TIMEOUT 30e6
#endif

#define MOTOR_BEEPS_PWM_ON 0.2
#define MOTOR_BEEPS_PWM_OFF 0.0

#ifndef DISABLE_PWM_PINS

void init_timer(TIM_TypeDef *TIMx, int period) {
  switch ((uint32_t)TIMx) {
#ifdef TIM1
  case (uint32_t)TIM1:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    break;
#endif
#ifdef TIM2
  case (uint32_t)TIM2:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    break;
#endif
#ifdef TIM3
  case (uint32_t)TIM3:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    break;
#endif
#ifdef TIM4
  case (uint32_t)TIM4:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);
    break;
#endif
#ifdef TIM5
  case (uint32_t)TIM5:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM5);
    break;
#endif
#ifdef TIM6
  case (uint32_t)TIM6:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);
    break;
#endif
#ifdef TIM7
  case (uint32_t)TIM7:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
    break;
#endif
#ifdef TIM8
  case (uint32_t)TIM8:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM8);
    break;
#endif
#ifdef TIM9
  case (uint32_t)TIM9:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
    break;
#endif
#ifdef TIM11
  case (uint32_t)TIM11:
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM11);
    break;
#endif
#ifdef TIM12
  case (uint32_t)TIM12:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM12);
    break;
#endif
#ifdef TIM13
  case (uint32_t)TIM13:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM13);
    break;
#endif
#ifdef TIM14
  case (uint32_t)TIM14:
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
    break;
#endif
  }

  LL_TIM_InitTypeDef tim_init;

  tim_init.Prescaler = PWM_DIVIDER - 1;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  tim_init.Autoreload = period;
  tim_init.ClockDivision = 0;
  tim_init.RepetitionCounter = 0;

  LL_TIM_Init(TIMx, &tim_init);
}

void motor_init(void) {

// timer clock enable
#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  init_timer(timer, PWMTOP);

  MOTOR_PINS

#undef MOTOR_PIN

  LL_TIM_OC_InitTypeDef tim_oc_init;
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.CompareValue = 0;

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;

#ifndef TIM14
#define TIM14 NULL
#endif

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)               \
  gpio_init.Pin = LL_GPIO_PIN_##pin;                                     \
  gpio_init.Alternate = pin_af;                                          \
  LL_GPIO_Init(GPIO##port, &gpio_init);                                  \
  LL_TIM_OC_Init(timer, LL_TIM_CHANNEL_CH##timer_channel, &tim_oc_init); \
  LL_TIM_EnableCounter(timer);                                           \
  if (timer != TIM14)                                                    \
    LL_TIM_EnableAllOutputs(timer);

  MOTOR_PINS

#undef MOTOR_PIN
}

void motor_beep(void) {
  if (flags.failsafe) {
    unsigned long time = timer_micros();
    if (!motorbeeptime)
      motorbeeptime = time;
    else if (time - motorbeeptime > MOTOR_BEEPS_TIMEOUT) {
      if (!beepon && (time % 2000000 < 125000)) {
        for (int i = 0; i <= 3; i++) {
          motor_set(i, MOTOR_BEEPS_PWM_ON);
          beepon = 1;
        }
      } else {
        for (int i = 0; i <= 3; i++) {
          motor_set(i, MOTOR_BEEPS_PWM_OFF);
          beepon = 0;
        }
      }
    }
  } else
    motorbeeptime = 0;
}

void motor_set(uint8_t number, float pwmf) {
  int pwm = pwmf * PWMTOP;

  if (pwm < 0)
    pwm = 0;
  if (pwm > PWMTOP)
    pwm = PWMTOP;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  case MOTOR_PIN_IDENT(port, pin):                         \
    timer->CCR##timer_channel = (uint16_t)pwm;             \
    break;

  switch (profile.motor.motor_pins[number]) {
    MOTOR_PINS
  default:
    // handle error;
    break;
  };

#undef MOTOR_PIN
}

#else
// pwm pins disabled
void motor_init(void) {
}

void motor_set(uint8_t number, float pwm) {
}

#endif

#endif // end USE_PWM_DRIVER
