#include "drv_motor.h"

#include <stdbool.h>
#include <string.h>

#include "drv_dma.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "flight/control.h"
#include "profile.h"
#include "project.h"
#include "util/util.h"

#if (defined(STM32F4) || defined(STM32F7) || defined(STM32H7)) && defined(USE_DSHOT_DMA_DRIVER)

#define DSHOT_TIME profile.motor.dshot_time
#define DSHOT_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_TIME * 1000 - 1))

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

#define DSHOT_MAX_PORT_COUNT 3
#define DSHOT_DMA_BUFFER_SIZE 48

typedef struct {
  motor_pin_ident_t id;

  GPIO_TypeDef *port;
  uint32_t pin;

  uint32_t dshot_port;
} motor_pin_t;

typedef struct {
  GPIO_TypeDef *gpio;

  uint32_t port_low;  // motor pins for BSRRL, for setting pins low
  uint32_t port_high; // motor pins for BSRRH, for setting pins high
} dshot_gpio_port_t;

uint8_t pwmdir = 0;

static uint8_t last_pwmdir = 0;
static uint32_t pwm_failsafe_time = 1;

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

static uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
static uint8_t gpio_port_count = 0;
static dshot_gpio_port_t gpio_ports[DSHOT_MAX_PORT_COUNT];
static volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

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
  LL_GPIO_ResetOutputPin(motor_pins[index].port, motor_pins[index].pin);

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (gpio_ports[i].gpio == motor_pins[index].port || i == gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      gpio_ports[i].gpio = motor_pins[index].port;
      gpio_ports[i].port_high |= motor_pins[index].pin;
      gpio_ports[i].port_low |= (motor_pins[index].pin << 16);

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
    for (uint32_t i = 0; i < DSHOT_DMA_BUFFER_SIZE; i = i + 3) {
      port_dma_buffer[j][i + 0] = gpio_ports[j].port_high;
      port_dma_buffer[j][i + 1] = gpio_ports[j].port_low;
      port_dma_buffer[j][i + 2] = gpio_ports[j].port_low;
    }
  }

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  // setup timer to 1/3 of the full bit time

  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_EnableARRPreload(TIM1);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 0;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &tim_oc_init);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  LL_DMA_DeInit(DMA2, LL_DMA_STREAM_6);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = LL_DMAMUX1_REQ_TIM1_CH1;
#else
  DMA_InitStructure.Channel = LL_DMA_CHANNEL_0;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&gpio_ports[0].gpio->BSRR;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)port_dma_buffer[0];
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
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

  NVIC_EnableIRQ(DMA2_Stream6_IRQn);

  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_6);
  LL_TIM_EnableCounter(TIM1);

  // set failsafetime so signal is off at start
  pwm_failsafe_time = time_micros() - 100000;
  pwmdir = FORWARD;
}

static void dshot_dma_setup_port(uint32_t index) {
  LL_DMA_ClearFlag_TC6(DMA2);
  LL_DMA_ClearFlag_HT6(DMA2);
  LL_DMA_ClearFlag_TE6(DMA2);

  dma_prepare_tx_memory((uint8_t *)port_dma_buffer[index], DSHOT_DMA_BUFFER_SIZE);

  DMA2_Stream6->PAR = (uint32_t)&gpio_ports[index].gpio->BSRR;
  DMA2_Stream6->M0AR = (uint32_t)&port_dma_buffer[index][0];
  DMA2_Stream6->NDTR = DSHOT_DMA_BUFFER_SIZE;

  gpio_ports[index].gpio->BSRR = gpio_ports[index].port_low;

  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_6);
  LL_TIM_EnableDMAReq_CC1(TIM1);
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

  memset((uint8_t *)port_dma_buffer, 0, DSHOT_MAX_PORT_COUNT * DSHOT_DMA_BUFFER_SIZE * sizeof(uint32_t));

  for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
    const uint32_t motor_high = (motor_pins[motor].pin);
    const uint32_t motor_low = (motor_pins[motor].pin << 16);

    const uint32_t port = motor_pins[motor].dshot_port;

    for (uint8_t i = 0; i < 16; i++) {
      const bool bit = dshot_packet[motor] & 0x8000;

      // start bit
      port_dma_buffer[port][i * 3 + 0] |= motor_high;

      // for 1 hold the line high for two timeunits
      port_dma_buffer[port][i * 3 + 1] |= bit ? motor_high : motor_low;

      // return line to low
      port_dma_buffer[port][i * 3 + 2] |= motor_low;

      dshot_packet[motor] <<= 1;
    }
  }

  dshot_dma_phase = gpio_port_count;
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
  LL_DMA_ClearFlag_TC6(DMA2);

  LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_6);
  LL_TIM_DisableDMAReq_CC1(TIM1);

  dshot_dma_phase--;

  if (dshot_dma_phase) {
    // not yet idle, more work to do
    dshot_dma_setup_port(dshot_dma_phase - 1);
  }
}

#endif