#include <math.h>

#include "control.h"
#include "defines.h"
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
  case (uint32_t)TIM1:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    break;
  case (uint32_t)TIM2:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    break;
  case (uint32_t)TIM3:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    break;
#ifdef F4
  case (uint32_t)TIM4:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    break;
  case (uint32_t)TIM5:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    break;
#endif
  case (uint32_t)TIM6:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    break;
  case (uint32_t)TIM7:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    break;
#ifdef F4
  case (uint32_t)TIM8:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    break;
  case (uint32_t)TIM9:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    break;
  case (uint32_t)TIM11:
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
    break;
  case (uint32_t)TIM12:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    break;
  case (uint32_t)TIM13:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
    break;
#endif
  case (uint32_t)TIM14:
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
    break;
  }

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  TIM_TimeBaseStructure.TIM_Prescaler = PWM_DIVIDER - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
}

void motor_init(void) {

// timer clock enable
#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  init_timer(timer, PWMTOP);

  MOTOR_PINS

#undef MOTOR_PIN

  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.OCState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.CompareValue = 0;

  LL_GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)   \
  GPIO_InitStructure.Pin = LL_GPIO_PIN_##pin;                \
  LL_GPIO_Init(GPIO##port, &GPIO_InitStructure);             \
  GPIO_PinAFConfig(GPIO##port, GPIO_PinSource##pin, pin_af); \
  TIM_OC##timer_channel##Init(timer, &TIM_OCInitStructure);  \
  TIM_Cmd(timer, ENABLE);                                    \
  if (timer != TIM14)                                        \
    TIM_CtrlPWMOutputs(timer, ENABLE);

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
