#include "driver/motor.h"

#include <stdbool.h>
#include <string.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/util.h"

#define DSHOT_TIME profile.motor.dshot_time
#define DSHOT_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_TIME * 1000) - 1)

#define DSHOT_CMD_BEEP1 1
#define DSHOT_CMD_BEEP2 2
#define DSHOT_CMD_BEEP3 3
#define DSHOT_CMD_BEEP4 4
#define DSHOT_CMD_BEEP5 5 // 5 currently uses the same tone as 4 in BLHeli_S.

#define DSHOT_CMD_ROTATE_NORMAL 20
#define DSHOT_CMD_ROTATE_REVERSE 21

#define DSHOT_MAX_PORT_COUNT 3
#define DSHOT_DMA_BUFFER_SIZE (3 * (16 + 2))

#define DSHOT_DIR_CHANGE_IDLE_TIME_US 10000
#define DSHOT_DIR_CHANGE_CMD_TIME_US 1000

typedef enum {
  DIR_CHANGE_START,
  DIR_CHANGE_DELAY,
  DIR_CHANGE_CMD,
  DIR_CHANGE_STOP,
} dir_change_state_t;

typedef struct {
  GPIO_TypeDef *port;
  uint32_t pin;

  uint32_t dshot_port;
} dshot_pin_t;

typedef struct {
  GPIO_TypeDef *gpio;

  uint32_t port_low;  // motor pins for BSRRL, for setting pins low
  uint32_t port_high; // motor pins for BSRRH, for setting pins high

  uint32_t timer_channel;
  dma_device_t dma_device;
} dshot_gpio_port_t;

static bool dir_change_done = true;
static motor_direction_t motor_dir = MOTOR_FORWARD;

static uint32_t pwm_failsafe_time = 1;
static uint32_t dir_change_time = 0;

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

static uint16_t dshot_packet[MOTOR_PIN_MAX]; // 16bits dshot data for 4 motors
static uint8_t gpio_port_count = 0;
static dshot_gpio_port_t gpio_ports[DSHOT_MAX_PORT_COUNT] = {
    {
        .timer_channel = LL_TIM_CHANNEL_CH1,
        .dma_device = DMA_DEVICE_TIM1_CH1,
    },
    {
        .timer_channel = LL_TIM_CHANNEL_CH3,
        .dma_device = DMA_DEVICE_TIM1_CH3,
    },
    {
        .timer_channel = LL_TIM_CHANNEL_CH4,
        .dma_device = DMA_DEVICE_TIM1_CH4,
    },
};
static volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

static void dshot_init_motor_pin(uint32_t index) {
  dshot_pins[index].port = gpio_pin_defs[target.motor_pins[index]].port;
  dshot_pins[index].pin = gpio_pin_defs[target.motor_pins[index]].pin;
  dshot_pins[index].dshot_port = 0;

  LL_GPIO_InitTypeDef gpio_init;
  gpio_init.Mode = LL_GPIO_MODE_OUTPUT;
  gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  gpio_init.Pull = LL_GPIO_PULL_NO;
  gpio_init.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  gpio_init.Pin = dshot_pins[index].pin;
  LL_GPIO_Init(dshot_pins[index].port, &gpio_init);
  LL_GPIO_ResetOutputPin(dshot_pins[index].port, dshot_pins[index].pin);

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (gpio_ports[i].gpio == dshot_pins[index].port || i == gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      gpio_ports[i].gpio = dshot_pins[index].port;
      gpio_ports[i].port_high |= dshot_pins[index].pin;
      gpio_ports[i].port_low |= (dshot_pins[index].pin << 16);

      dshot_pins[index].dshot_port = i;

      if (i + 1 > gpio_port_count) {
        gpio_port_count = i + 1;
      }

      break;
    }
  }
}

static void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;
  LL_TIM_OC_Init(TIM1, port->timer_channel, &tim_oc_init);
  LL_TIM_OC_EnablePreload(TIM1, port->timer_channel);

  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#ifdef STM32H7
  DMA_InitStructure.PeriphRequest = dma->request;
#else
  DMA_InitStructure.Channel = dma->channel;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&port->gpio->BSRR;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)port_dma_buffer[0];
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  LL_DMA_Init(dma->port, dma->stream_index, &DMA_InitStructure);

  interrupt_enable(dma->irq, DMA_PRIORITY);

  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);
}

static void dshot_enable_dma_request(uint32_t timer_channel) {
  switch (timer_channel) {
  case LL_TIM_CHANNEL_CH1:
    LL_TIM_EnableDMAReq_CC1(TIM1);
    break;
  case LL_TIM_CHANNEL_CH3:
    LL_TIM_EnableDMAReq_CC3(TIM1);
    break;
  case LL_TIM_CHANNEL_CH4:
    LL_TIM_EnableDMAReq_CC4(TIM1);
    break;
  default:
    break;
  }
}

static void dshot_disable_dma_request(uint32_t timer_channel) {
  switch (timer_channel) {
  case LL_TIM_CHANNEL_CH1:
    LL_TIM_DisableDMAReq_CC1(TIM1);
    break;
  case LL_TIM_CHANNEL_CH3:
    LL_TIM_DisableDMAReq_CC3(TIM1);
    break;
  case LL_TIM_CHANNEL_CH4:
    LL_TIM_DisableDMAReq_CC4(TIM1);
    break;
  default:
    break;
  }
}

void motor_dshot_init() {
  gpio_port_count = 0;

  rcc_enable(RCC_APB2_GRP1(TIM1));
  rcc_enable(RCC_AHB1_GRP1(DMA2));

  // setup timer to 1/3 of the full bit time

  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_EnableARRPreload(TIM1);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_init_gpio_port(&gpio_ports[j]);

    for (uint32_t i = 0; i < DSHOT_DMA_BUFFER_SIZE; i += 3) {
      port_dma_buffer[j][i + 0] = gpio_ports[j].port_low;
      port_dma_buffer[j][i + 1] = gpio_ports[j].port_low;
      port_dma_buffer[j][i + 2] = gpio_ports[j].port_low;
    }
  }

  LL_TIM_EnableCounter(TIM1);

  // set failsafetime so signal is off at start
  pwm_failsafe_time = time_micros() - 100000;
  motor_dir = MOTOR_FORWARD;
}

static void dshot_dma_setup_port(uint32_t index) {
  dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_clear_flag_tc(dma->port, dma->stream_index);

  dma->stream->PAR = (uint32_t)&port->gpio->BSRR;
  dma->stream->M0AR = (uint32_t)&port_dma_buffer[index][0];
  dma->stream->NDTR = DSHOT_DMA_BUFFER_SIZE;

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  dshot_enable_dma_request(port->timer_channel);
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

static void make_packet_all(uint16_t value, bool telemetry) {
  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    make_packet(profile.motor.motor_pins[i], value, telemetry);
  }
}

// make dshot dma packet, then fire
static void dshot_dma_start() {
  motor_wait_for_ready();

  for (uint32_t j = 0; j < gpio_port_count; j++) {
    // set all ports to low before and after the packet
    port_dma_buffer[j][0] = gpio_ports[j].port_low;
    port_dma_buffer[j][1] = gpio_ports[j].port_low;
    port_dma_buffer[j][2] = gpio_ports[j].port_low;

    port_dma_buffer[j][16 * 3 + 0] = gpio_ports[j].port_low;
    port_dma_buffer[j][16 * 3 + 1] = gpio_ports[j].port_low;
    port_dma_buffer[j][16 * 3 + 2] = gpio_ports[j].port_low;
  }

  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t j = 0; j < gpio_port_count; j++) {
      port_dma_buffer[j][(i + 1) * 3 + 0] = gpio_ports[j].port_high; // start bit
      port_dma_buffer[j][(i + 1) * 3 + 1] = 0;                       // actual bit, set below
      port_dma_buffer[j][(i + 1) * 3 + 2] = gpio_ports[j].port_low;  // return line to low
    }

    for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
      const uint32_t port = dshot_pins[motor].dshot_port;
      const uint32_t motor_high = (dshot_pins[motor].pin);
      const uint32_t motor_low = (dshot_pins[motor].pin << 16);

      const bool bit = dshot_packet[motor] & 0x8000;

      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      port_dma_buffer[port][(i + 1) * 3 + 1] |= bit ? motor_high : motor_low;

      dshot_packet[motor] <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)port_dma_buffer, sizeof(port_dma_buffer));

  dshot_dma_phase = gpio_port_count;
  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_dma_setup_port(j);
  }
}

void motor_dshot_wait_for_ready() {
#ifdef STM32F4
  while (dshot_dma_phase != 0 || spi_dma_is_ready(SPI_PORT1) == 0)
#else
  while (dshot_dma_phase != 0)
#endif
    __NOP();
}

void motor_dshot_write(float *values) {
  if (dir_change_done) {
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      uint16_t value = 0;

      if (values[i] >= 0.0f) {
        const float pwm = constrainf(values[i], 0.0f, 1.0f);
        value = mapf(pwm, 0.0f, 1.0f, 48, 2047);
      } else {
        value = 0;
      }

      if (flags.failsafe && !flags.motortest_override) {
        if (!pwm_failsafe_time) {
          pwm_failsafe_time = time_micros();
        } else if (time_micros() - pwm_failsafe_time > 4000000) {
          // 1s after failsafe we turn off the signal for safety
          // this means the escs won't rearm correctly after 2 secs of signal lost
          // usually the quad should be gone by then
          value = 0;
        }
      } else {
        pwm_failsafe_time = 0;
      }

      make_packet(profile.motor.motor_pins[i], value, false);
    }

    dshot_dma_start();
  } else {
    static uint8_t counter = 0;
    static dir_change_state_t state = DIR_CHANGE_START;

    switch (state) {
    case DIR_CHANGE_START: {
      if (counter < 100) {
        make_packet_all(0, false);
        dshot_dma_start();
        counter++;
      } else {
        state = DIR_CHANGE_DELAY;
        counter = 0;
      }
      dir_change_time = time_micros();
      break;
    }

    case DIR_CHANGE_DELAY:
      if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_IDLE_TIME_US) {
        break;
      }
      state = DIR_CHANGE_CMD;
      dir_change_time = time_micros();
      break;

    case DIR_CHANGE_CMD: {
      if ((time_micros() - dir_change_time) < DSHOT_DIR_CHANGE_CMD_TIME_US) {
        break;
      }

      const uint16_t value = motor_dir == MOTOR_REVERSE ? DSHOT_CMD_ROTATE_REVERSE : DSHOT_CMD_ROTATE_NORMAL;
      if (counter < 10) {
        make_packet_all(value, true);
        dshot_dma_start();
        counter++;
      } else {
        state = DIR_CHANGE_STOP;
        counter = 0;
      }
      dir_change_time = time_micros();
      break;
    }

    case DIR_CHANGE_STOP:
      dir_change_done = true;
      state = DIR_CHANGE_START;
      break;
    }
  }
}

void motor_dshot_set_direction(motor_direction_t dir) {
  if (dir_change_done) {
    motor_dir = dir;
    dir_change_done = false;
  }
}

bool motor_dshot_direction_change_done() {
  return dir_change_done;
}

void motor_dshot_beep() {
  static uint32_t last_time = 0;
  const uint32_t time = time_millis() - last_time;

  static uint8_t beep_command = DSHOT_CMD_BEEP1;
  make_packet_all(beep_command, true);
  dshot_dma_start();

  if (time >= 500) {
    beep_command++;
    if (beep_command > DSHOT_CMD_BEEP5) {
      beep_command = DSHOT_CMD_BEEP1;
    }

    last_time = time_millis();
  }
}

void dshot_dma_isr(dma_device_t dev) {
  for (uint32_t j = 0; j < gpio_port_count; j++) {
    if (gpio_ports[j].dma_device != dev) {
      continue;
    }

    const dma_stream_def_t *dma = &dma_stream_defs[dev];
    dma_clear_flag_tc(dma->port, dma->stream_index);

    LL_DMA_DisableStream(dma->port, dma->stream_index);
    dshot_disable_dma_request(gpio_ports[j].timer_channel);

    dshot_dma_phase--;
    break;
  }
}