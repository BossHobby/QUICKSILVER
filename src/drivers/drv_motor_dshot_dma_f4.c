
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

#include <stdbool.h>

#include <stm32f4xx_ll_bus.h>
#include <stm32f4xx_ll_dma.h>
#include <stm32f4xx_ll_tim.h>

#include "control.h"
#include "defines.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "profile.h"
#include "project.h"
#include "util.h"

#ifdef F4
typedef struct {
  __IO uint32_t MODER;   /*!< GPIO port mode register,               Address offset: 0x00      */
  __IO uint32_t OTYPER;  /*!< GPIO port output type register,        Address offset: 0x04      */
  __IO uint32_t OSPEEDR; /*!< GPIO port output speed register,       Address offset: 0x08      */
  __IO uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  __IO uint32_t IDR;     /*!< GPIO port input data register,         Address offset: 0x10      */
  __IO uint32_t ODR;     /*!< GPIO port output data register,        Address offset: 0x14      */
  __IO uint16_t BSRRL;   /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  __IO uint16_t BSRRH;   /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  __IO uint32_t LCKR;    /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  __IO uint32_t AFR[2];  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} dshot_gpio_t;

// use your own struct to use BSRR as a 16-bit register
static volatile dshot_gpio_t *gpioA = (dshot_gpio_t *)GPIOA;
static volatile dshot_gpio_t *gpioB = (dshot_gpio_t *)GPIOB;
static volatile dshot_gpio_t *gpioC = (dshot_gpio_t *)GPIOC;

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

void make_packet(uint8_t number, uint16_t value, bool telemetry);

#ifndef FORWARD
#define FORWARD 0
#define REVERSE 1
#endif

void motor_init() {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;

#define MOTOR_PIN(port, pin, pin_af, timer, timer_channel) \
  gpio_init.Pin = LL_GPIO_PIN_##pin;                       \
  LL_GPIO_Init(GPIO##port, &gpio_init);                    \
  if (GPIO##port == GPIOA) {                               \
    DSHOT_GPIO_A = 1;                                      \
    dshot_portA_low |= (LL_GPIO_PIN_##pin);                \
    dshot_portA_high |= (LL_GPIO_PIN_##pin << 16);         \
  } else if (GPIO##port == GPIOB) {                        \
    DSHOT_GPIO_B = 1;                                      \
    dshot_portB_low |= (LL_GPIO_PIN_##pin);                \
    dshot_portB_high |= (LL_GPIO_PIN_##pin << 16);         \
  } else if (GPIO##port == GPIOC) {                        \
    DSHOT_GPIO_C = 1;                                      \
    dshot_portC_low |= (LL_GPIO_PIN_##pin);                \
    dshot_portC_high |= (LL_GPIO_PIN_##pin << 16);         \
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
  // TIM1_CH1 DMA2_STREAM_6/CH0: reset output if data=0		at TIM1 update - CompareValue = 0
  // TIM1_CH2 DMA2_STREAM_6/CH0: reset output if data=0		at T0H timing
  // TIM1_CH3 DMA2_STREAM_6/CH0: reset all output			at T1H timing

  LL_TIM_InitTypeDef tim_init;
  LL_TIM_OC_InitTypeDef tim_oc_init;

  LL_TIM_StructInit(&tim_init);
  LL_TIM_OC_StructInit(&tim_oc_init);

  // TIM1 Periph clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* Time base configuration */
  tim_init.Autoreload = DSHOT_BIT_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_DisableARRPreload(TIM1);

  /* Timing Mode configuration: Channel 1 */
  tim_oc_init.OCMode = LL_TIM_OCMODE_FROZEN;
  tim_oc_init.OCState = LL_TIM_OCSTATE_DISABLE;
  tim_oc_init.CompareValue = 0;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &tim_oc_init);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  /* Timing Mode configuration: Channel 2 */
  tim_oc_init.OCMode = LL_TIM_OCMODE_FROZEN;
  tim_oc_init.OCState = LL_TIM_OCSTATE_DISABLE;
  tim_oc_init.CompareValue = DSHOT_T0H_TIME;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &tim_oc_init);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH2);

  /* Timing Mode configuration: Channel 3 */
  tim_oc_init.OCMode = LL_TIM_OCMODE_FROZEN;
  tim_oc_init.OCState = LL_TIM_OCSTATE_DISABLE;
  tim_oc_init.CompareValue = DSHOT_T1H_TIME;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &tim_oc_init);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH3);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA2 Stream6_Channel0 configuration ----------------------------------------------*/
  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_6);
  DMA_InitStructure.Channel = LL_DMA_CHANNEL_0;
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&gpioA->BSRRL;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)portA_buffer;
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = 48;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(DMA2, LL_DMA_STREAM_6, &DMA_InitStructure);

  LL_TIM_EnableDMAReq_CC3(TIM1);
  LL_TIM_EnableDMAReq_CC2(TIM1);
  LL_TIM_EnableDMAReq_CC1(TIM1);

  NVIC_EnableIRQ(DMA2_Stream6_IRQn);

  // enable DMA2 Stream6 transfer complete interrupt
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);

  // set failsafetime so signal is off at start
  pwm_failsafe_time = timer_micros() - 100000;
  pwmdir = FORWARD;
}

static void dshot_dma_stream_enable() {
  LL_DMA_ClearFlag_TC6(DMA2);
  LL_DMA_ClearFlag_HT6(DMA2);
  LL_DMA_ClearFlag_TE6(DMA2);

  DMA2_Stream6->NDTR = 48;
  TIM1->SR = 0;

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);

  LL_TIM_EnableDMAReq_CC3(TIM1);
  LL_TIM_EnableDMAReq_CC2(TIM1);
  LL_TIM_EnableDMAReq_CC1(TIM1);

  LL_TIM_SetCounter(TIM1, DSHOT_BIT_TIME);
  LL_TIM_EnableCounter(TIM1);
}

void dshot_dma_portA() {
  DMA2_Stream6->PAR = (uint32_t)&gpioA->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portA_buffer;
  dshot_dma_stream_enable();
}

void dshot_dma_portB() {
  DMA2_Stream6->PAR = (uint32_t)&gpioB->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portB_buffer;
  dshot_dma_stream_enable();
}

void dshot_dma_portC() {
  DMA2_Stream6->PAR = (uint32_t)&gpioC->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)portC_buffer;
  dshot_dma_stream_enable();
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
      motor_data_portA[i] |= (LL_GPIO_PIN_##pin << 16);       \
    else if (GPIO##port == GPIOB)                             \
      motor_data_portB[i] |= (LL_GPIO_PIN_##pin << 16);       \
    else if (GPIO##port == GPIOC)                             \
      motor_data_portC[i] |= (LL_GPIO_PIN_##pin << 16);       \
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
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);

  LL_TIM_DisableDMAReq_CC3(TIM1);
  LL_TIM_DisableDMAReq_CC2(TIM1);
  LL_TIM_DisableDMAReq_CC1(TIM1);

  LL_DMA_ClearFlag_TC6(DMA2);
  LL_TIM_DisableCounter(TIM1);

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
