#include <math.h>

#include "defines.h"
#include "drv_motor.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef USE_ESC_DRIVER

// working motor range (microseconds)
#define ESC_MIN 1180
#define ESC_MAX 1860

// esc throttle off setting (microseconds
#define ESC_OFF 900

// zero = no signal ( microseconds)
#define ESC_FAILSAFE 0

// invert = signal after fets (may need 1k pullup resistor)
// commented = signal straight from CPU pins
#define ESC_INVERT_SIGNAL

// enable preload
#define ENABLE_PRELOAD

//#define ENABLE_ONESHOT

#define ENABLE_ONESHOT42

// signal repetition frequency (hertz) min 185 max 500
#define ESC_FREQ_PPM 500

// Oneshot default 1000Hz ( 1khz) 500 - 4000
#define ESC_FREQ_ONESHOT 1000

// do not change below

// set ESC_FREQ based on PPM or Oneshot use
#undef ESC_FREQ
#define ESC_FREQ ESC_FREQ_PPM

#ifdef ENABLE_ONESHOT
#undef ESC_FREQ
#define ESC_FREQ ESC_FREQ_ONESHOT
#endif

#ifdef ENABLE_ONESHOT42
#undef ENABLE_ONESHOT
#undef ESC_FREQ
#define ESC_FREQ ESC_FREQ_ONESHOT
#endif

#ifndef SYS_CLOCK_FREQ_HZ
#define SYS_CLOCK_FREQ_HZ 48000000
#warning SYS_CLOCK_FREQ_HZ not present
#endif

// max pulse width in microseconds (auto calculated)
#define ESC_uS ((float)1000000.0f / ESC_FREQ)

#define PWMTOP ((SYS_CLOCK_FREQ_HZ / ESC_FREQ) - 1)
#define PWM_DIVIDER 1

#if (PWMTOP < 1000)
#error PWM FREQUENCY TOO HIGH
#endif

#if (PWMTOP > 65535)
#undef PWMTOP
#undef PWM_DIVIDER
#define PWMTOP (((SYS_CLOCK_FREQ_HZ / 4) / ESC_FREQ) - 1)
#define PWM_DIVIDER 4
//	#warning USING DIVIDER
#endif

#if (PWMTOP > 65535)
//	#undef PWMTOP
//	#undef PWM_DIVIDER
//	#define PWMTOP 6000
//	#define PWM_DIVIDER 1
#error PWM FREQUENCY TOO LOW
#endif

#ifndef DISABLE_PWM_PINS
unsigned long pwm_failsafe_time = 1;
extern int failsafe;
extern int onground;
extern profile_t profile;

void init_timer(TIM_TypeDef *TIMx, int period) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  TIM_TimeBaseStructure.TIM_Prescaler = PWM_DIVIDER - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
#ifdef ENABLE_PRELOAD
  TIM_ARRPreloadConfig(TIMx, ENABLE);
#endif
}

void motor_init(void) {

// timer clock enable
#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_##timer, ENABLE);  \
  init_timer(timer, PWMTOP);

  MOTOR_PINS

#undef MOTOR_PIN

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
#ifdef ESC_INVERT_SIGNAL
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
#else
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
#endif
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  TIM_OCInitStructure.TIM_Pulse = 0;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)   \
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_##pin;              \
  GPIO_Init(GPIO##port, &GPIO_InitStructure);                \
  GPIO_PinAFConfig(GPIO##port, GPIO_PinSource##pin, pin_af); \
  TIM_OC##timer_channel##Init(timer, &TIM_OCInitStructure);  \
  TIM_Cmd(timer, ENABLE);                                    \
  if (timer != TIM14)                                        \
    TIM_CtrlPWMOutputs(timer, ENABLE);

  MOTOR_PINS

#undef MOTOR_PIN
}

void motor_set(uint8_t number, float pwmf) {

  if (pwmf < 0.0f)
    pwmf = 0.0f;
  if (pwmf > 1.0f)
    pwmf = 1.0f;

  pwmf = mapf(pwmf, 0.0, 1.0, (float)ESC_MIN / ESC_uS, (float)ESC_MAX / ESC_uS);

  if (onground)
    pwmf = (float)ESC_OFF / ESC_uS;

  if (failsafe) {
    if (!pwm_failsafe_time) {
      pwm_failsafe_time = gettime();
    } else {
      // 100mS after failsafe we turn off the signal (for safety while flashing)
      if (gettime() - pwm_failsafe_time > 100000) {
        pwmf = (float)ESC_FAILSAFE / ESC_uS;
      }
    }

  } else {
    pwm_failsafe_time = 0;
  }

#ifdef ENABLE_ONESHOT
  pwmf = pwmf / 8;
#endif

#ifdef ENABLE_ONESHOT42
  pwmf = pwmf / 24;
#endif

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

#endif // end USE_ESC_DRIVER
