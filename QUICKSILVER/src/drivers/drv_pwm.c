
#include "drv_pwm.h"

#include <math.h>

#include "defines.h"
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

#ifdef PWM_PA0
#define PWM_PA0_PIN GPIO_Pin_0
#define PWM_PA0_PORT GPIOA
#define PWM_PA0_AF GPIO_AF_2
#define PWM_PA0_PINSOURCE GPIO_PinSource0
#define PWM_PA0_TIMER TIM2
#define PWM_PA0_CHANNEL CH1
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PA1
#define PWM_PA1_PIN GPIO_Pin_1
#define PWM_PA1_PORT GPIOA
#define PWM_PA1_AF GPIO_AF_2
#define PWM_PA1_PINSOURCE GPIO_PinSource1
#define PWM_PA1_TIMER TIM2
#define PWM_PA1_CHANNEL CH2
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PA2
#define PWM_PA2_PIN GPIO_Pin_2
#define PWM_PA2_PORT GPIOA
#ifdef F405
#define PWM_PA2_AF GPIO_AF_TIM2
#endif
#ifdef F0
#define PWM_PA3_AF GPIO_AF_2
#endif
#define PWM_PA2_PINSOURCE GPIO_PinSource2
#define PWM_PA2_TIMER TIM2
#define PWM_PA2_CHANNEL CH3
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PA3
#define PWM_PA3_PIN GPIO_Pin_3
#define PWM_PA3_PORT GPIOA
#ifdef F405
#define PWM_PA3_AF GPIO_AF_TIM2
#endif
#ifdef F0
#define PWM_PA3_AF GPIO_AF_2
#endif
#define PWM_PA3_PINSOURCE GPIO_PinSource3
#define PWM_PA3_TIMER TIM2
#define PWM_PA3_CHANNEL CH4
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PA4
#define PWM_PA4_PIN GPIO_Pin_4
#define PWM_PA4_PORT GPIOA
#ifdef F405
#define PWM_PA4_AF GPIO_AF_TIM14
#endif
#ifdef F0
#define PWM_PA4_AF GPIO_AF_4
#endif
#define PWM_PA4_PINSOURCE GPIO_PinSource4
#define PWM_PA4_TIMER TIM14
#define PWM_PA4_CHANNEL CH1
#ifndef ENABLE_TIM14
#define ENABLE_TIM14
#endif
#endif

#ifdef PWM_PA5
#define PWM_PA5_PIN GPIO_Pin_5
#define PWM_PA5_PORT GPIOA
#define PWM_PA5_AF GPIO_AF_2
#define PWM_PA5_PINSOURCE GPIO_PinSource5
#define PWM_PA5_TIMER TIM2
#define PWM_PA5_CHANNEL CH1
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PA6
#define PWM_PA6_PIN GPIO_Pin_6
#define PWM_PA6_PORT GPIOA
#ifdef F405
#define PWM_PA6_AF GPIO_AF_TIM3
#endif
#ifdef F0
#define PWM_PA6_AF GPIO_AF_1
#endif
#define PWM_PA6_PINSOURCE GPIO_PinSource6
#define PWM_PA6_TIMER TIM3
#define PWM_PA6_CHANNEL CH1
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PA7
#define PWM_PA7_PIN GPIO_Pin_7
#define PWM_PA7_PORT GPIOA
#ifdef F405
#define PWM_PA7_AF GPIO_AF_TIM3
#endif
#ifdef F0
#define PWM_PA7_AF GPIO_AF_1
#endif
#define PWM_PA7_PINSOURCE GPIO_PinSource7
#define PWM_PA7_TIMER TIM3
#define PWM_PA7_CHANNEL CH2
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PA8
#define PWM_PA8_PIN GPIO_Pin_8
#define PWM_PA8_PORT GPIOA
#define PWM_PA8_AF GPIO_AF_2
#define PWM_PA8_PINSOURCE GPIO_PinSource8
#define PWM_PA8_TIMER TIM1
#define PWM_PA8_CHANNEL CH1
#ifndef ENABLE_TIM1
#define ENABLE_TIM1
#endif
#endif

#ifdef PWM_PA9
#define PWM_PA9_PIN GPIO_Pin_9
#define PWM_PA9_PORT GPIOA
#define PWM_PA9_AF GPIO_AF_2
#define PWM_PA9_PINSOURCE GPIO_PinSource9
#define PWM_PA9_TIMER TIM1
#define PWM_PA9_CHANNEL CH2
#ifndef ENABLE_TIM1
#define ENABLE_TIM1
#endif
#endif

#ifdef PWM_PA10
#define PWM_PA10_PIN GPIO_Pin_10
#define PWM_PA10_PORT GPIOA
#define PWM_PA10_AF GPIO_AF_2
#define PWM_PA10_PINSOURCE GPIO_PinSource10
#define PWM_PA10_TIMER TIM1
#define PWM_PA10_CHANNEL CH3
#ifndef ENABLE_TIM1
#define ENABLE_TIM1
#endif
#endif

#ifdef PWM_PA11
#define PWM_PA11_PIN GPIO_Pin_11
#define PWM_PA11_PORT GPIOA
#define PWM_PA11_AF GPIO_AF_2
#define PWM_PA11_PINSOURCE GPIO_PinSource11
#define PWM_PA11_TIMER TIM1
#define PWM_PA11_CHANNEL CH4
#ifndef ENABLE_TIM1
#define ENABLE_TIM1
#endif
#endif

#ifdef PWM_PB0
#define PWM_PB0_PIN GPIO_Pin_0
#define PWM_PB0_PORT GPIOB
#ifdef F405
#define PWM_PB0_AF GPIO_AF_TIM3
#endif
#ifdef F0
#define PWM_PB0
_AF GPIO_AF_1
#endif
#define PWM_PB0_PINSOURCE GPIO_PinSource0
#define PWM_PB0_TIMER TIM3
#define PWM_PB0_CHANNEL CH3
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PB1
#define PWM_PB1_PIN GPIO_Pin_1
#define PWM_PB1_PORT GPIOB
#ifdef F405
#define PWM_PB1_AF GPIO_AF_TIM3
#endif
#ifdef F0
#define PWM_PB1_AF GPIO_AF_1
#endif
#define PWM_PB1_PINSOURCE GPIO_PinSource1
#define PWM_PB1_TIMER TIM3
#define PWM_PB1_CHANNEL CH4
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PB6
#define PWM_PB6_PIN GPIO_Pin_6
#define PWM_PB6_PORT GPIOB
#define PWM_PB6_AF GPIO_AF_TIM4
#define PWM_PB6_PINSOURCE GPIO_PinSource6
#define PWM_PB6_TIMER TIM4
#define PWM_PB6_CHANNEL CH1
#ifndef ENABLE_TIM4
#define ENABLE_TIM4
#endif
#endif

#ifdef PWM_PB7
#define PWM_PB7_PIN GPIO_Pin_7
#define PWM_PB7_PORT GPIOB
#define PWM_PB7_AF GPIO_AF_TIM4
#define PWM_PB7_PINSOURCE GPIO_PinSource7
#define PWM_PB7_TIMER TIM4
#define PWM_PB7_CHANNEL CH2
#ifndef ENABLE_TIM4
#define ENABLE_TIM4
#endif
#endif

#ifdef PWM_PB8
#define PWM_PB8_PIN GPIO_Pin_8
#define PWM_PB8_PORT GPIOB
#define PWM_PB8_AF GPIO_AF_TIM4
#define PWM_PB8_PINSOURCE GPIO_PinSource8
#define PWM_PB8_TIMER TIM4
#define PWM_PB8_CHANNEL CH3
#ifndef ENABLE_TIM4
#define ENABLE_TIM4
#endif
#endif

#ifdef PWM_PB10
#define PWM_PB10_PIN GPIO_Pin_10
#define PWM_PB10_PORT GPIOB
#define PWM_PB10_AF GPIO_AF_TIM3
#define PWM_PB10_PINSOURCE GPIO_PinSource10
#define PWM_PB10_TIMER TIM2
#define PWM_PB10_CHANNEL CH3
#ifndef ENABLE_TIM2
#define ENABLE_TIM2
#endif
#endif

#ifdef PWM_PC6
#define PWM_PC6_PIN GPIO_Pin_6
#define PWM_PC6_PORT GPIOC
#define PWM_PC6_AF GPIO_AF_TIM3
#define PWM_PC6_PINSOURCE GPIO_PinSource6
#define PWM_PC6_TIMER TIM3
#define PWM_PC6_CHANNEL CH1
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif

#endif
#ifdef PWM_PC7
#define PWM_PC7_PIN GPIO_Pin_7
#define PWM_PC7_PORT GPIOC
#define PWM_PC7_AF GPIO_AF_TIM3
#define PWM_PC7_PINSOURCE GPIO_PinSource7
#define PWM_PC7_TIMER TIM3
#define PWM_PC7_CHANNEL CH2
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PC8 //F4 only
#define PWM_PC8_PIN GPIO_Pin_8
#define PWM_PC8_PORT GPIOC
#define PWM_PC8_AF GPIO_AF_TIM3
#define PWM_PC8_PINSOURCE GPIO_PinSource8
#define PWM_PC8_TIMER TIM3
#define PWM_PC8_CHANNEL CH3
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifdef PWM_PC9 //F4 only
#define PWM_PC9_PIN GPIO_Pin_9
#define PWM_PC9_PORT GPIOC
#define PWM_PC9_AF GPIO_AF_TIM3
#define PWM_PC9_PINSOURCE GPIO_PinSource9
#define PWM_PC9_TIMER TIM3
#define PWM_PC9_CHANNEL CH4
#ifndef ENABLE_TIM3
#define ENABLE_TIM3
#endif
#endif

#ifndef DISABLE_PWM_PINS

    void
    init_timer(TIM_TypeDef *TIMx, int period);

TIM_OCInitTypeDef TIM_OCInitStructure;

void pwm_init(void) {

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

// timer clock enable
#ifdef ENABLE_TIM1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // PA8 , PA9
  init_timer(TIM1, PWMTOP);
#endif

#ifdef ENABLE_TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // PA0 , PA1
  init_timer(TIM2, PWMTOP);
#endif

#ifdef ENABLE_TIM3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // PA6 , 7 , PB1
  init_timer(TIM3, PWMTOP);
#endif

#ifdef ENABLE_TIM4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // PB6, PB7, PB8
  init_timer(TIM4, PWMTOP);
#endif

#ifdef ENABLE_TIM14
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
  init_timer(TIM14, PWMTOP);
#endif

#ifdef ENABLE_TIM16
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE); // PB8
  init_timer(TIM16, PWMTOP);
#endif

  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  TIM_OCInitStructure.TIM_Pulse = 0;

#ifdef PWM_PA0
  GPIO_InitStructure.GPIO_Pin = PWM_PA0_PIN;
  GPIO_Init(PWM_PA0_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA0_PORT, PWM_PA0_PINSOURCE, PWM_PA0_AF);

#if (PWM_PA0_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA0_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA0_CHANNEL == CH2)
  TIM_OC2Init(PWM1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA0_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA0_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA0_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA0_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PA1
  GPIO_InitStructure.GPIO_Pin = PWM_PA1_PIN;
  GPIO_Init(PWM_PA1_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA1_PORT, PWM_PA1_PINSOURCE, PWM_PA1_AF);

#if (PWM_PA1_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA1_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA1_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA1_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA1_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA2
  GPIO_InitStructure.GPIO_Pin = PWM_PA2_PIN;
  GPIO_Init(PWM_PA2_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA2_PORT, PWM_PA2_PINSOURCE, PWM_PA2_AF);

#if (PWM_PA2_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA2_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA2_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA2_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA2_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA2_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA2_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA2_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA3
  GPIO_InitStructure.GPIO_Pin = PWM_PA3_PIN;
  GPIO_Init(PWM_PA3_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA3_PORT, PWM_PA3_PINSOURCE, PWM_PA3_AF);

#if (PWM_PA3_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA3_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA3_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA3_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA3_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA3_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA3_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA3_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA4
  GPIO_InitStructure.GPIO_Pin = PWM_PA4_PIN;
  GPIO_Init(PWM_PA4_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA4_PORT, PWM_PA4_PINSOURCE, PWM_PA4_AF);

#if (PWM_PA4_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA4_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA4_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA4_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA4_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA4_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA4_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA4_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA5
  GPIO_InitStructure.GPIO_Pin = PWM_PA5_PIN;
  GPIO_Init(PWM_PA5_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA5_PORT, PWM_PA5_PINSOURCE, PWM_PA5_AF);

#if (PWM_PA5_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA5_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA5_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA5_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA5_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA5_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA5_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA5_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA6
  GPIO_InitStructure.GPIO_Pin = PWM_PA6_PIN;
  GPIO_Init(PWM_PA6_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA6_PORT, PWM_PA6_PINSOURCE, PWM_PA6_AF);

#if (PWM_PA6_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA6_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA6_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA6_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA6_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PA7
  GPIO_InitStructure.GPIO_Pin = PWM_PA7_PIN;
  GPIO_Init(PWM_PA7_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA7_PORT, PWM_PA7_PINSOURCE, PWM_PA7_AF);

#if (PWM_PA7_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA7_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA7_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA7_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA7_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PA8
  GPIO_InitStructure.GPIO_Pin = PWM_PA8_PIN;
  GPIO_Init(PWM_PA8_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA8_PORT, PWM_PA8_PINSOURCE, PWM_PA8_AF);

#if (PWM_PA8_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA8_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA8_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA8_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA8_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA9
  GPIO_InitStructure.GPIO_Pin = PWM_PA9_PIN;
  GPIO_Init(PWM_PA9_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA9_PORT, PWM_PA9_PINSOURCE, PWM_PA9_AF);

#if (PWM_PA9_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA9_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA9_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA9_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA9_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA10
  GPIO_InitStructure.GPIO_Pin = PWM_PA10_PIN;
  GPIO_Init(PWM_PA10_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA10_PORT, PWM_PA10_PINSOURCE, PWM_PA10_AF);

#if (PWM_PA10_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA10_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA10_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA10_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA10_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PA11
  GPIO_InitStructure.GPIO_Pin = PWM_PA11_PIN;
  GPIO_Init(PWM_PA11_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PA11_PORT, PWM_PA11_PINSOURCE, PWM_PA11_AF);

#if (PWM_PA11_CHANNEL == CH1)
  TIM_OC1Init(PWM_PA11_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA11_CHANNEL == CH2)
  TIM_OC2Init(PWM_PA11_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA11_CHANNEL == CH3)
  TIM_OC3Init(PWM_PA11_TIMER, &TIM_OCInitStructure);
#elif (PWM_PA11_CHANNEL == CH4)
  TIM_OC4Init(PWM_PA11_TIMER, &TIM_OCInitStructure);
#endif
#endif

#ifdef PWM_PB0
  GPIO_InitStructure.GPIO_Pin = PWM_PB0_PIN;
  GPIO_Init(PWM_PB0_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB0_PORT, PWM_PB0_PINSOURCE, PWM_PB0_AF);

#if (PWM_PB0_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB0_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB0_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB0_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB0_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB0_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB0_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB0_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PB1
  GPIO_InitStructure.GPIO_Pin = PWM_PB1_PIN;
  GPIO_Init(PWM_PB1_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB1_PORT, PWM_PB1_PINSOURCE, PWM_PB1_AF);

#if (PWM_PB1_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB1_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB1_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB1_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB1_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB1_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PB6
  GPIO_InitStructure.GPIO_Pin = PWM_PB6_PIN;
  GPIO_Init(PWM_PB6_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB6_PORT, PWM_PB6_PINSOURCE, PWM_PB6_AF);

#if (PWM_PB6_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB6_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB6_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB6_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB6_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PB7
  GPIO_InitStructure.GPIO_Pin = PWM_PB7_PIN;
  GPIO_Init(PWM_PB7_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB7_PORT, PWM_PB7_PINSOURCE, PWM_PB7_AF);

#if (PWM_PB7_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB7_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB7_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB7_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB7_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PB8
  GPIO_InitStructure.GPIO_Pin = PWM_PB8_PIN;
  GPIO_Init(PWM_PB8_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB8_PORT, PWM_PB8_PINSOURCE, PWM_PB8_AF);

#if (PWM_PB8_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB8_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB8_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB8_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB8_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PB10
  GPIO_InitStructure.GPIO_Pin = PWM_PB10_PIN;
  GPIO_Init(PWM_PB10_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PB10_PORT, PWM_PB10_PINSOURCE, PWM_PB10_AF);

#if (PWM_PB10_CHANNEL == CH1)
  TIM_OC1Init(PWM_PB10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB10_CHANNEL == CH2)
  TIM_OC2Init(PWM_PB10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB10_CHANNEL == CH3)
  TIM_OC3Init(PWM_PB10_TIMER, &TIM_OCInitStructure);
#elif (PWM_PB10_CHANNEL == CH4)
  TIM_OC4Init(PWM_PB10_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PC6
  GPIO_InitStructure.GPIO_Pin = PWM_PC6_PIN;
  GPIO_Init(PWM_PC6_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PC6_PORT, PWM_PC6_PINSOURCE, PWM_PC6_AF);

#if (PWM_PC6_CHANNEL == CH1)
  TIM_OC1Init(PWM_PC6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC6_CHANNEL == CH2)
  TIM_OC2Init(PWM_PC6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC6_CHANNEL == CH3)
  TIM_OC3Init(PWM_PC6_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC6_CHANNEL == CH4)
  TIM_OC4Init(PWM_PC6_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PC7
  GPIO_InitStructure.GPIO_Pin = PWM_PC7_PIN;
  GPIO_Init(PWM_PC7_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PC7_PORT, PWM_PC7_PINSOURCE, PWM_PC7_AF);

#if (PWM_PC7_CHANNEL == CH1)
  TIM_OC1Init(PWM_PC7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC7_CHANNEL == CH2)
  TIM_OC2Init(PWM_PC7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC7_CHANNEL == CH3)
  TIM_OC3Init(PWM_PC7_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC7_CHANNEL == CH4)
  TIM_OC4Init(PWM_PC7_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PC8
  GPIO_InitStructure.GPIO_Pin = PWM_PC8_PIN;
  GPIO_Init(PWM_PC8_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PC8_PORT, PWM_PC8_PINSOURCE, PWM_PC8_AF);

#if (PWM_PC8_CHANNEL == CH1)
  TIM_OC1Init(PWM_PC8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC8_CHANNEL == CH2)
  TIM_OC2Init(PWM_PC8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC8_CHANNEL == CH3)
  TIM_OC3Init(PWM_PC8_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC8_CHANNEL == CH4)
  TIM_OC4Init(PWM_PC8_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef PWM_PC9
  GPIO_InitStructure.GPIO_Pin = PWM_PC9_PIN;
  GPIO_Init(PWM_PC9_PORT, &GPIO_InitStructure);

  GPIO_PinAFConfig(PWM_PC9_PORT, PWM_PC9_PINSOURCE, PWM_PC9_AF);

#if (PWM_PC9_CHANNEL == CH1)
  TIM_OC1Init(PWM_PC9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC9_CHANNEL == CH2)
  TIM_OC2Init(PWM_PC9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC9_CHANNEL == CH3)
  TIM_OC3Init(PWM_PC9_TIMER, &TIM_OCInitStructure);
#elif (PWM_PC9_CHANNEL == CH4)
  TIM_OC4Init(PWM_PC9_TIMER, &TIM_OCInitStructure);
#endif

#endif

#ifdef ENABLE_TIM1
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
#endif

#ifdef ENABLE_TIM2
  TIM_Cmd(TIM2, ENABLE);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
#endif

#ifdef ENABLE_TIM3
  TIM_Cmd(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
#endif

#ifdef ENABLE_TIM4
  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
#endif

#ifdef ENABLE_TIM14
  TIM_Cmd(TIM14, ENABLE);
  // t14 does not support pwm out enable
  //TIM_CtrlPWMOutputs(TIM14, ENABLE);
#endif

#ifdef ENABLE_TIM16
  TIM_Cmd(TIM16, ENABLE);
  TIM_CtrlPWMOutputs(TIM16, ENABLE);
#endif
}

void init_timer(TIM_TypeDef *TIMx, int period) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

  TIM_TimeBaseStructure.TIM_Prescaler = PWM_DIVIDER - 1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = period;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
}

extern int failsafe;
unsigned long motorbeeptime = 0;
int beepon = 0;
#include "drv_time.h"

#ifndef MOTOR_BEEPS_TIMEOUT
// default value if not defined elsewhere
#define MOTOR_BEEPS_TIMEOUT 30e6
#endif

#define MOTOR_BEEPS_PWM_ON 0.2
#define MOTOR_BEEPS_PWM_OFF 0.0

void motorbeep(void) {
  if (failsafe) {
    unsigned long time = gettime();
    if (!motorbeeptime)
      motorbeeptime = time;
    else if (time - motorbeeptime > MOTOR_BEEPS_TIMEOUT) {
      if (!beepon && (time % 2000000 < 125000)) {
        for (int i = 0; i <= 3; i++) {
          pwm_set(i, MOTOR_BEEPS_PWM_ON);
          beepon = 1;
        }
      } else {
        for (int i = 0; i <= 3; i++) {
          pwm_set(i, MOTOR_BEEPS_PWM_OFF);
          beepon = 0;
        }
      }
    }
  } else
    motorbeeptime = 0;
}

void pwm_set(uint8_t number, float pwmf) {

  int pwm = pwmf * PWMTOP;

  if (pwm < 0)
    pwm = 0;
  if (pwm > PWMTOP)
    pwm = PWMTOP;

  switch (number) {
  case 0:
#ifdef MOTOR0_PIN_PA0
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA1
    TIM2->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA2
    TIM2->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA3
    TIM2->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA4
    TIM14->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA5
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA6
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA7
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA8
    TIM1->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA9
    TIM1->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA10
    TIM1->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PA11
    TIM1->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB0
    TIM3->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB1
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB6
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB7
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB8
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PB10
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PC6
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PC7
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PC8
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR0_PIN_PC9
    TIM3->CCR3 = (uint16_t)pwm;
#endif
    break;

  case 1:
#ifdef MOTOR1_PIN_PA0
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA1
    TIM2->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA2
    TIM2->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA3
    TIM2->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA4
    TIM14->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA5
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA6
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA7
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA8
    TIM1->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA9
    TIM1->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA10
    TIM1->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PA11
    TIM1->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB0
    TIM3->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB1
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB6
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB7
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB8
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PB10
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PC6
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PC7
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PC8
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR1_PIN_PC9
    TIM3->CCR3 = (uint16_t)pwm;
#endif
    break;

  case 2:
#ifdef MOTOR2_PIN_PA0
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA1
    TIM2->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA2
    TIM2->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA3
    TIM2->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA4
    TIM14->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA5
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA6
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA7
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA8
    TIM1->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA9
    TIM1->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA10
    TIM1->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PA11
    TIM1->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB0
    TIM3->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB1
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB6
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB7
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB8
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PB10
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PC6
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PC7
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PC8
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR2_PIN_PC9
    TIM3->CCR3 = (uint16_t)pwm;
#endif
    break;

  case 3:
#ifdef MOTOR3_PIN_PA0
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA1
    TIM2->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA2
    TIM2->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA3
    TIM2->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA4
    TIM14->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA5
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA6
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA7
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA8
    TIM1->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA9
    TIM1->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA10
    TIM1->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PA11
    TIM1->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB0
    TIM3->CCR3 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB1
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB6
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB7
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB8
    TIM4->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PB10
    TIM2->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PC6
    TIM3->CCR2 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PC7
    TIM3->CCR1 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PC8
    TIM3->CCR4 = (uint16_t)pwm;
#endif
#ifdef MOTOR3_PIN_PC9
    TIM3->CCR3 = (uint16_t)pwm;
#endif

    break;

  default:
    // handle error;
    //
    break;
  }
}

#else
// pwm pins disabled
void pwm_init(void) {
}

void pwm_set(uint8_t number, float pwm) {
}

#endif

#endif // end USE_PWM_DRIVER
