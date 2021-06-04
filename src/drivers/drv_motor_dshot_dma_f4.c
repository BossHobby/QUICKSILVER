
// History & props :)
// Origional F0 Silverware Dshot driver for H101_dual firmware. Written by Markus Gritsch.
// Modified by JazzMac to support DMA transfer
// Ported to F4 by NotFastEnuf
// Supporting runtime configuration by Hanfer
// Modified to run on only one DMA stream instead of three by NFE

// this DMA driver was origionally done with the reference to http://www.cnblogs.com/shangdawei/p/4762035.html

// No throttle jitter, no min/max calibration, just pure digital goodness :)
// FC with motor pins on multiple gpio ports will fire ports sequentially.
// A dshot600 frame takes about ~30us, dshot300 frame ~60us, and dshot150 frame ~120us
// It is important to consider how many motor gpio ports are used in a target and to allow time for each port to fire within set looptime.
// examples:  motor pins on 3 gpio ports - 8k looptime - dshot600 = safe 90us of dshot data fits in 125us loop
// 			  motor pins on 3 gpio ports - 8k looptime - dshot300 = unsafe 180us of dshot data DOES NOT fit in 125us loop
// 			  motor pins on 3 gpio ports - 4k looptime - dshot300 = safe 180us of dshot data fits in 250us loop

// USE AT YOUR OWN RISK. ALWAYS REMOVE PROPS WHEN TESTING.

#include "control.h"
#include "defines.h"
#include "drv_motor.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef F4
//watch variables to be removed after testing
//static volatile GPIO_TypeDef *gpioA = GPIOA;
//static volatile GPIO_TypeDef *gpioB = GPIOB;
//static volatile GPIO_TypeDef *gpioC = GPIOC;

// Tim_1 is running at 84mhz with APB2 clock currently configured at 42MHZ
// clock cycles per bit for a bit timing period of 1.67us
#define DSHOT_BIT_TIME ((PWM_CLOCK_FREQ_HZ / 1000 / profile.motor.dshot_time) - 1)
#define DSHOT_T0H_TIME (DSHOT_BIT_TIME * 0.30 + 0.05)
#define DSHOT_T1H_TIME (DSHOT_BIT_TIME * 0.60 + 0.05)

#ifdef USE_DSHOT_DMA_DRIVER

#ifdef THREE_D_THROTTLE
#error "Not tested with THREE_D_THROTTLE config option"
#endif

#ifdef INVERTED_ENABLE
#ifndef BIDIRECTIONAL
#error INVERTED_ENABLE is on but not BIDIRECTIONAL in dshot driver
#endif
#endif

static uint8_t DSHOT_GPIO_A = 0;
static uint8_t DSHOT_GPIO_B = 0;
static uint8_t DSHOT_GPIO_C = 0;

//sum = total number of dshot GPIO ports
#define DSHOT_PORT_COUNT (DSHOT_GPIO_A + DSHOT_GPIO_B + DSHOT_GPIO_C)

extern profile_t profile;

int pwmdir = 0;
int last_pwmdir = 0;

static unsigned long pwm_failsafe_time = 1;

volatile int dshot_dma_phase = 0;  // 0:idle, 1: also idles in interrupt handler as single phase gets called by dshot_dma_start(), 2: & 3: handle remaining phases
volatile uint16_t dshot_packet[4]; // 16bits dshot data for 4 motors

volatile uint32_t motor_data_portA[16] = {0}; //motor pins at portA with commanded '0' bit at T0H timing
volatile uint32_t motor_data_portB[16] = {0}; //motor pins at portB with commanded '0' bit at T0H timing
volatile uint32_t motor_data_portC[16] = {0}; //motor pins at portC with commanded '0' bit at T0H timing

volatile uint32_t dshot_portA_low = 0;  // sum of all motor pins at portA
volatile uint32_t dshot_portB_low = 0;  // sum of all motor pins at portB
volatile uint32_t dshot_portC_low = 0;  // sum of all motor pins at portC
volatile uint32_t dshot_portA_high = 0; // shifted motor pins at portA for T1H timing
volatile uint32_t dshot_portB_high = 0; // shifted motor pins at portB for T1H timing
volatile uint32_t dshot_portC_high = 0; // shifted motor pins at portC for T1H timing

volatile uint32_t portA_buffer[48] = {0}; // dma buffers
volatile uint32_t portB_buffer[48] = {0};
volatile uint32_t portC_buffer[48] = {0};

typedef enum { false,
               true } bool;

void make_packet(uint8_t number, uint16_t value, bool telemetry);

#ifndef FORWARD
#define FORWARD 0
#define REVERSE 1
#endif

void motor_init() {
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_##pin;            \
  GPIO_Init(GPIO##port, &GPIO_InitStructure);              \
  if (GPIO##port == GPIOA) {                               \
    DSHOT_GPIO_A = 1;                                      \
    dshot_portA_low |= (GPIO_Pin_##pin);                   \
    dshot_portA_high |= (GPIO_Pin_##pin << 16);            \
  } else if (GPIO##port == GPIOB) {                        \
    DSHOT_GPIO_B = 1;                                      \
    dshot_portB_low |= (GPIO_Pin_##pin);                   \
    dshot_portB_high |= (GPIO_Pin_##pin << 16);            \
  } else if (GPIO##port == GPIOC) {                        \
    DSHOT_GPIO_C = 1;                                      \
    dshot_portC_low |= (GPIO_Pin_##pin);                   \
    dshot_portC_high |= (GPIO_Pin_##pin << 16);            \
  }

  MOTOR_PINS

#undef MOTOR_PIN

  for (int i = 0; i < 48; i = i + 3) {
    portA_buffer[i] = dshot_portA_low;
    portB_buffer[i] = dshot_portB_low;
    portC_buffer[i] = dshot_portC_low;
  }

  for (int i = 2; i < 48; i = i + 3) {
    portA_buffer[i] = dshot_portA_high;
    portB_buffer[i] = dshot_portB_high;
    portC_buffer[i] = dshot_portC_high;
  }

  // DShot timer/DMA2 init
  // TIM1_UP: set all output to HIGH		                at TIM1 update - no dma event
  // TIM1_CH1 DMA2_STREAM_6/CH0: reset output if data=0		at TIM1 update - TIM_Pulse = 0
  // TIM1_CH2 DMA2_STREAM_6/CH0: reset output if data=0		at T0H timing
  // TIM1_CH3 DMA2_STREAM_6/CH0: reset all output			at T1H timing

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  // TIM1 Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = DSHOT_BIT_TIME;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM1, DISABLE);

  /* Timing Mode configuration: Channel 1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);

  /* Timing Mode configuration: Channel 2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = DSHOT_T0H_TIME;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);

  /* Timing Mode configuration: Channel 3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = DSHOT_T1H_TIME;
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);

  DMA_InitTypeDef DMA_InitStructure;

  DMA_StructInit(&DMA_InitStructure);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  /* DMA2 Stream6_Channel0 configuration ----------------------------------------------*/
  DMA_DeInit(DMA2_Stream6);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPIOA->BSRRL;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)portA_buffer;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = 48;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);

  TIM_DMACmd(TIM1, TIM_DMA_CC3 | TIM_DMA_CC2 | TIM_DMA_CC1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* configure DMA2 Stream 6 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_Init(&NVIC_InitStructure);
  /* enable DMA2 Stream6 transfer complete interrupt */
  DMA_ITConfig(DMA2_Stream6, DMA_IT_TC, ENABLE);

  // set failsafetime so signal is off at start
  pwm_failsafe_time = timer_micros() - 100000;
  pwmdir = FORWARD;
}

void dshot_dma_portA() {
  DMA2_Stream6->PAR = (uint32_t)&GPIOA->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portA_buffer;
  DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6);
  DMA2_Stream6->NDTR = 48;
  TIM1->SR = 0;
  DMA_Cmd(DMA2_Stream6, ENABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC3 | TIM_DMA_CC2 | TIM_DMA_CC1, ENABLE);
  TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
  TIM_Cmd(TIM1, ENABLE);
}

void dshot_dma_portB() {
  DMA2_Stream6->PAR = (uint32_t)&GPIOB->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portB_buffer;
  DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6);
  DMA2_Stream6->NDTR = 48;
  TIM1->SR = 0;
  DMA_Cmd(DMA2_Stream6, ENABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC3 | TIM_DMA_CC2 | TIM_DMA_CC1, ENABLE);
  TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
  TIM_Cmd(TIM1, ENABLE);
}

void dshot_dma_portC() {
  DMA2_Stream6->PAR = (uint32_t)&GPIOC->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portC_buffer;
  DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6);
  DMA2_Stream6->NDTR = 48;
  TIM1->SR = 0;
  DMA_Cmd(DMA2_Stream6, ENABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC3 | TIM_DMA_CC2 | TIM_DMA_CC1, ENABLE);
  TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
  TIM_Cmd(TIM1, ENABLE);
}

// make dshot packet
void make_packet(uint8_t number, uint16_t value, bool telemetry) {
  uint16_t packet = (value << 1) | (telemetry ? 1 : 0); // Here goes telemetry bit
  // compute checksum
  uint16_t csum = 0;
  uint16_t csum_data = packet;

  for (uint8_t i = 0; i < 3; ++i) {
    csum ^= csum_data; // xor data by nibbles
    csum_data >>= 4;
  }

  csum &= 0xf;
  // append checksum
  dshot_packet[number] = (packet << 4) | csum;
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  uint32_t time = timer_micros();

  // wait maximum a LOOPTIME for dshot dma to complete
  while ((dshot_dma_phase != 0 || spi_dma_is_ready(SPI_PORT1) == 0) && (timer_micros() - time) < state.looptime * 1e6f)
    ;

  if (dshot_dma_phase != 0 || spi_dma_is_ready(SPI_PORT1) == 0)
    return; // skip this dshot command

  // generate dshot dma packet
  for (uint8_t i = 0; i < 16; i++) {
    motor_data_portA[i] = 0;
    motor_data_portB[i] = 0;
    motor_data_portC[i] = 0;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel)    \
  if (!(dshot_packet[MOTOR_PIN_IDENT(port, pin)] & 0x8000)) { \
    if (GPIO##port == GPIOA)                                  \
      motor_data_portA[i] |= (GPIO_Pin_##pin << 16);          \
    else if (GPIO##port == GPIOB)                             \
      motor_data_portB[i] |= (GPIO_Pin_##pin << 16);          \
    else if (GPIO##port == GPIOC)                             \
      motor_data_portC[i] |= (GPIO_Pin_##pin << 16);          \
  }

    MOTOR_PINS

#undef MOTOR_PIN

    dshot_packet[0] <<= 1;
    dshot_packet[1] <<= 1;
    dshot_packet[2] <<= 1;
    dshot_packet[3] <<= 1;
  }

  for (int i = 1, j = 0; i < 48 && j < 16; i += 3, j++) {
    portA_buffer[i] = motor_data_portA[j];
    portB_buffer[i] = motor_data_portB[j];
    portC_buffer[i] = motor_data_portC[j];
  }

  dshot_dma_phase = DSHOT_PORT_COUNT;

  TIM1->ARR = DSHOT_BIT_TIME;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = DSHOT_T0H_TIME;
  TIM1->CCR3 = DSHOT_T1H_TIME;

  if (DSHOT_GPIO_A == 1)
    dshot_dma_portA();
  else if (DSHOT_GPIO_B == 1)
    dshot_dma_portB();
  else if (DSHOT_GPIO_C == 1)
    dshot_dma_portC();
}

void motor_set(uint8_t number, float pwm) {
  // if ( number > 3 ) failloop(5);
  if (number > 3)
    return;

  if (pwm < 0.0f) {
    pwm = 0.0;
  }
  if (pwm > 0.999f) {
    pwm = 0.999;
  }

  uint16_t value = 0;

#ifdef BIDIRECTIONAL

  if (pwmdir == FORWARD) {
    // maps 0.0 .. 0.999 to 48 + IDLE_OFFSET .. 1047
    value = 48 + (profile.motor.digital_idle * 10) + (uint16_t)(pwm * (1000 - (profile.motor.digital_idle * 10)));
  } else if (pwmdir == REVERSE) {
    // maps 0.0 .. 0.999 to 1048 + IDLE_OFFSET .. 2047
    value = 1048 + (profile.motor.digital_idle * 10) + (uint16_t)(pwm * (1000 - (profile.motor.digital_idle * 10)));
  }

#else

  // maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
  value = 48 + (profile.motor.digital_idle * 20) + (uint16_t)(pwm * (2001 - (profile.motor.digital_idle * 20)));

#endif
  if (flags.on_ground || !flags.arm_state || (flags.motortest_override && pwm < 0.002f) || ((rx_aux_on(AUX_MOTOR_TEST)) && pwm < 0.002f)) { //turn off the slow motors during turtle or motortest
    value = 0;                                                                                                                                           // stop the motors
  }

  if (flags.failsafe && !flags.motortest_override) {
    if (!pwm_failsafe_time) {
      pwm_failsafe_time = timer_micros();
    } else {
      // 1s after failsafe we turn off the signal for safety
      // this means the escs won't rearm correctly after 2 secs of signal lost
      // usually the quad should be gone by then
      if (timer_micros() - pwm_failsafe_time > 4000000) {
        value = 0;
        //  return;    -   esc reboots are annoying
      }
    }
  } else {
    pwm_failsafe_time = 0;
  }

#ifdef BIDIRECTIONAL
  make_packet(profile.motor.motor_pins[number], value, false);
#else
  if (pwmdir == last_pwmdir) { //make a regular packet
    make_packet(profile.motor.motor_pins[number], value, false);
  } else { //make a series of dshot command packets
    static uint16_t counter;
    if (counter <= 10000) {
      counter++;
      if (pwmdir == REVERSE)
        value = 21; //DSHOT_CMD_ROTATE_REVERSE 21
      if (pwmdir == FORWARD)
        value = 20;        //DSHOT_CMD_ROTATE_NORMAL 20
      if (counter <= 8000) //override to disarmed for a few cycles just since case blheli wants that
        make_packet(profile.motor.motor_pins[number], 0, false);
      if (counter > 8000 && counter <= 8060) //send the command 6 times plus a few extra times for good measure
        make_packet(profile.motor.motor_pins[number], value, true);
      if (counter > 8600) //override to disarmed for a few cycles just since case blheli wants that
        make_packet(profile.motor.motor_pins[number], 0, false);
    }
    if (counter == 10001) {
      counter = 0;
      last_pwmdir = pwmdir;
    }
  }
#endif

  if (number == 3) {
    dshot_dma_start();
  }
}

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#ifndef MOTOR_BEEPS_TIMEOUT
#define MOTOR_BEEPS_TIMEOUT 1e6
#endif

void motor_beep() {
  static unsigned long motor_beep_time = 0;
  if (flags.failsafe) {
    unsigned long time = timer_micros();
    if (motor_beep_time == 0) {
      motor_beep_time = time;
    }
    const unsigned long delta_time = time - motor_beep_time;
    if (delta_time > MOTOR_BEEPS_TIMEOUT) {
      uint8_t beep_command = 0;
      if (delta_time % 2000000 < 250000) {
        beep_command = DSHOT_CMD_BEEP1;
      } else if (delta_time % 2000000 < 500000) {
        beep_command = DSHOT_CMD_BEEP3;
      } else if (delta_time % 2000000 < 750000) {
        beep_command = DSHOT_CMD_BEEP2;
      } else if (delta_time % 2000000 < 1000000) {
        beep_command = DSHOT_CMD_BEEP4;
      }

      if (beep_command != 0) {
        make_packet(0, beep_command, true);
        make_packet(1, beep_command, true);
        make_packet(2, beep_command, true);
        make_packet(3, beep_command, true);
        dshot_dma_start();
      }
    }
  } else {
    motor_beep_time = 0;
  }
}

#if defined(USE_DSHOT_DMA_DRIVER)

void DMA2_Stream6_IRQHandler(void) {
  DMA_Cmd(DMA2_Stream6, DISABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC3 | TIM_DMA_CC2 | TIM_DMA_CC1, DISABLE);
  DMA_ClearITPendingBit(DMA2_Stream6, DMA_IT_TCIF6);
  TIM_Cmd(TIM1, DISABLE);

  switch (dshot_dma_phase) {
  case 3: //has to be port B here because we are in 3 phase mode and port A runs first in dshot_dma_start()
    dshot_dma_phase = 2;
    dshot_dma_portB();
    return;
  case 2: //could be port B or C here
    dshot_dma_phase = 1;
    //if 3 phase, B already fired in case 3, its just C left ... if 2 phase AC or BC, then first phase has fired and its just C left
    if (DSHOT_PORT_COUNT == 3 || (DSHOT_PORT_COUNT == 2 && DSHOT_GPIO_B == 0) || (DSHOT_PORT_COUNT == 2 && DSHOT_GPIO_A == 0))
      dshot_dma_portC();
    //last possible GPIO phase combo is 2 phase AB, where A has already fired off and now we need B
    else
      dshot_dma_portB();
    return;
  case 1:
    dshot_dma_phase = 0;
    return;
  default:
    dshot_dma_phase = 0;
    break;
  }
}
#endif

#endif

#endif
