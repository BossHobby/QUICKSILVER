
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

#include "drv_dma.h"
#include "drv_gpio.h"
#include "drv_motor.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

#if (defined(STM32F4) || defined(STM32F7) || defined(STM32H7)) && defined(USE_DSHOT_DMA_DRIVER)

// Tim_1 is running at 84mhz with APB2 clock currently configured at 42MHZ
// clock cycles per bit for a bit timing period of 1.67us
#define DSHOT_BIT_TIME ((PWM_CLOCK_FREQ_HZ / 1000 / profile.motor.dshot_time) - 1)
#define DSHOT_T0H_TIME (DSHOT_BIT_TIME * 0.35 + 0.05)
#define DSHOT_T1H_TIME (DSHOT_BIT_TIME * 0.70 + 0.05)

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

#define DSHOT_MAX_PORT_COUNT 3

typedef struct {
  motor_pin_ident_t id;

  GPIO_TypeDef *port;
  uint32_t pin;

  uint32_t dshot_port;
} motor_pin_t;

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

typedef struct {
  dshot_gpio_t *gpio;

  uint32_t port_low;  // sum of all motor pins of this port
  uint32_t port_high; // shifted motor pins of this port for T1H timing

  uint32_t motor_data[16]; // motor pins at this port with commanded '0' bit at T0H timing
} dshot_gpio_port_t;

extern profile_t profile;

uint8_t pwmdir = 0;

static uint8_t last_pwmdir = 0;
static uint32_t pwm_failsafe_time = 1;

volatile uint32_t dshot_dma_phase = 0;       // 0: idle, 1 - (gpio_port_count + 1): handle port n
static uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
static uint8_t gpio_port_count = 0;
static dshot_gpio_port_t gpio_ports[DSHOT_MAX_PORT_COUNT];
static volatile uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][48];

#define MOTOR_PIN(_port, _pin, pin_af, timer, timer_channel) \
  {                                                          \
      .id = MOTOR_PIN_IDENT(_port, _pin),                    \
      .port = GPIO##_port,                                   \
      .pin = LL_GPIO_PIN_##_pin,                             \
      .dshot_port = 0,                                       \
  },

static motor_pin_t motor_pins[MOTOR_PIN_MAX] = {MOTOR_PINS};

#undef MOTOR_PIN

static void dshot_init_motor_pin(uint32_t index) {
  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = motor_pins[index].pin;
  LL_GPIO_Init(motor_pins[index].port, &gpio_init);

  dshot_gpio_t *dshot_gpio = (dshot_gpio_t *)motor_pins[index].port;

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (gpio_ports[i].gpio == dshot_gpio || i == gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      gpio_ports[i].gpio = dshot_gpio;
      gpio_ports[i].port_low |= motor_pins[index].pin;
      gpio_ports[i].port_high |= (motor_pins[index].pin << 16);

      motor_pins[index].dshot_port = i;

      if (i + 1 > gpio_port_count) {
        gpio_port_count = i + 1;
      }

      break;
    }
  }
}

void motor_init() {
  gpio_port_count = 0;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < gpio_port_count; j++) {
    for (uint32_t i = 0; i < 48; i = i + 3) {
      port_dma_buffer[j][i] = gpio_ports[j].port_low;
    }

    for (uint32_t i = 2; i < 48; i = i + 3) {
      port_dma_buffer[j][i] = gpio_ports[j].port_high;
    }
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
#ifdef STM32H7
  // TODO: fix me, other channels right now do nothing,
  // switch to solution where a single channel runs at BIT_TIME/3
  DMA_InitStructure.PeriphRequest = LL_DMAMUX1_REQ_TIM1_CH1;
#else
  DMA_InitStructure.Channel = LL_DMA_CHANNEL_0;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&gpio_ports[0].gpio->BSRRL;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)port_dma_buffer[0];
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
  pwm_failsafe_time = time_micros() - 100000;
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

static void dshot_dma_setup_port(uint32_t index) {
  dma_prepare_tx_memory((uint8_t *)port_dma_buffer[index], 48);
  DMA2_Stream6->PAR = (uint32_t)&gpio_ports[index].gpio->BSRRL;
  DMA2_Stream6->M0AR = (uint32_t)port_dma_buffer[index];
  dshot_dma_stream_enable();
}

// make dshot packet
static void make_packet(uint8_t number, uint16_t value, bool telemetry) {
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
static void dshot_dma_start() {
  motor_wait_for_ready();

  // generate dshot dma packet
  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t port = 0; port < gpio_port_count; port++) {
      gpio_ports[port].motor_data[i] = 0;
    }

    for (uint8_t j = 0; j < MOTOR_PIN_MAX; j++) {
      if (!(dshot_packet[j] & 0x8000)) {
        gpio_ports[motor_pins[j].dshot_port].motor_data[i] |= (motor_pins[j].pin << 16);
      }
    }

    dshot_packet[0] <<= 1;
    dshot_packet[1] <<= 1;
    dshot_packet[2] <<= 1;
    dshot_packet[3] <<= 1;
  }

  for (int i = 1, j = 0; i < 48 && j < 16; i += 3, j++) {
    for (uint32_t port = 0; port < gpio_port_count; port++) {
      port_dma_buffer[port][i] = gpio_ports[port].motor_data[j];
    }
  }

  dshot_dma_phase = gpio_port_count;

  TIM1->ARR = DSHOT_BIT_TIME;
  TIM1->CCR1 = 0;
  TIM1->CCR2 = DSHOT_T0H_TIME;
  TIM1->CCR3 = DSHOT_T1H_TIME;

  dshot_dma_setup_port(dshot_dma_phase - 1);
}

void motor_wait_for_ready() {
#ifdef STM32F4
  while (dshot_dma_phase != 0 || spi_dma_is_ready(SPI_PORT1) == 0)
#else
  while (dshot_dma_phase != 0)
#endif
    __WFI();
}

void motor_set(uint8_t number, float pwm) {
  if (number > 3)
    return;

  if (pwm < 0.0f) {
    pwm = 0.0;
  }
  if (pwm > 0.999f) {
    pwm = 0.999;
  }

  // maps 0.0 .. 0.999 to 48 + IDLE_OFFSET * 2 .. 2047
  uint16_t value = 48 + (profile.motor.digital_idle * 20) + (uint16_t)(pwm * (2001 - (profile.motor.digital_idle * 20)));

  if (flags.on_ground || !flags.arm_state || (flags.motortest_override && pwm < 0.002f) || ((rx_aux_on(AUX_MOTOR_TEST)) && pwm < 0.002f)) { // turn off the slow motors during turtle or motortest
    value = 0;                                                                                                                              // stop the motors
  }

  if (flags.failsafe && !flags.motortest_override) {
    if (!pwm_failsafe_time) {
      pwm_failsafe_time = time_micros();
    } else {
      // 1s after failsafe we turn off the signal for safety
      // this means the escs won't rearm correctly after 2 secs of signal lost
      // usually the quad should be gone by then
      if (time_micros() - pwm_failsafe_time > 4000000) {
        value = 0;
        //  return;    -   esc reboots are annoying
      }
    }
  } else {
    pwm_failsafe_time = 0;
  }

  if (pwmdir == last_pwmdir) { // make a regular packet
    make_packet(profile.motor.motor_pins[number], value, false);
  } else { // make a series of dshot command packets
    static uint16_t counter = 0;
    if (counter <= 10000) {
      counter++;
      if (pwmdir == REVERSE)
        value = DSHOT_CMD_ROTATE_REVERSE;
      if (pwmdir == FORWARD)
        value = DSHOT_CMD_ROTATE_NORMAL;
      if (counter <= 8000) // override to disarmed for a few cycles just in case blheli wants that
        make_packet(profile.motor.motor_pins[number], 0, false);
      if (counter > 8000 && counter <= 8060) // send the command 6 times plus a few extra times for good measure
        make_packet(profile.motor.motor_pins[number], value, true);
      if (counter > 8600) // override to disarmed for a few cycles just in case blheli wants that
        make_packet(profile.motor.motor_pins[number], 0, false);
    }
    if (counter == 10001) {
      counter = 0;
      last_pwmdir = pwmdir;
    }
  }

  if (number == 3) {
    dshot_dma_start();
  }
}

void motor_beep() {
  const uint32_t time = time_millis();

  uint8_t beep_command = 0;
  if (time % 2000 <= 250) {
    beep_command = DSHOT_CMD_BEEP1;
  } else if (time % 2000 <= 500) {
    beep_command = DSHOT_CMD_BEEP3;
  } else if (time % 2000 <= 750) {
    beep_command = DSHOT_CMD_BEEP2;
  } else if (time % 2000 <= 1000) {
    beep_command = DSHOT_CMD_BEEP4;
  } else if (time % 2000 <= 1250) {
    beep_command = DSHOT_CMD_BEEP5;
  }

  if (beep_command != 0) {
    make_packet(0, beep_command, true);
    make_packet(1, beep_command, true);
    make_packet(2, beep_command, true);
    make_packet(3, beep_command, true);
    dshot_dma_start();
  }
}

void DMA2_Stream6_IRQHandler() {
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);

  LL_TIM_DisableDMAReq_CC3(TIM1);
  LL_TIM_DisableDMAReq_CC2(TIM1);
  LL_TIM_DisableDMAReq_CC1(TIM1);

  LL_DMA_ClearFlag_TC6(DMA2);
  LL_TIM_DisableCounter(TIM1);

  dshot_dma_phase--;

  if (dshot_dma_phase) {
    // not yet idle, more work to do
    dshot_dma_setup_port(dshot_dma_phase - 1);
  }
}

#endif