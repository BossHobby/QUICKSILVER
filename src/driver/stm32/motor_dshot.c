#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"
#include "driver/spi.h"

#ifdef USE_MOTOR_DSHOT

#define DSHOT_TIME profile.motor.dshot_time
#define DSHOT_SYMBOL_TIME (PWM_CLOCK_FREQ_HZ / (3 * DSHOT_TIME * 1000) - 1)

#define DSHOT_MAX_PORT_COUNT 3
#define DSHOT_DMA_BUFFER_SIZE (3 * 16)

typedef struct {
  gpio_port_t *port;
  uint32_t pin;

  uint32_t dshot_port;
} dshot_pin_t;

typedef struct {
  gpio_port_t *gpio;

  uint32_t port_low;  // motor pins for BSRRL, for setting pins low
  uint32_t port_high; // motor pins for BSRRH, for setting pins high

  uint32_t timer_channel;
  dma_device_t dma_device;
} dshot_gpio_port_t;

extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern motor_direction_t motor_dir;

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

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

static const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev) {
  return &gpio_ports[dev - DMA_DEVICE_TIM1_CH1];
}

static void dshot_init_motor_pin(uint32_t index) {
  gpio_config_t gpio_init;
  gpio_init.mode = GPIO_OUTPUT;
  gpio_init.output = GPIO_PUSHPULL;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.pull = GPIO_NO_PULL;
  gpio_pin_init(target.motor_pins[index], gpio_init);
  gpio_pin_reset(target.motor_pins[index]);

  dshot_pins[index].port = gpio_pin_defs[target.motor_pins[index]].port;
  dshot_pins[index].pin = gpio_pin_defs[target.motor_pins[index]].pin;
  dshot_pins[index].dshot_port = 0;

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

  dma_enable_rcc(port->dma_device);
  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
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
#ifndef STM32G4
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
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

  // setup timer to 1/3 of the full bit time
  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_DisableMasterSlaveMode(TIM1);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_init_gpio_port(&gpio_ports[j]);

    for (uint8_t i = 0; i < 16; i++) {
      port_dma_buffer[j][i * 3 + 0] = gpio_ports[j].port_high; // start bit
      port_dma_buffer[j][i * 3 + 1] = 0;                       // actual bit, set below
      port_dma_buffer[j][i * 3 + 2] = gpio_ports[j].port_low;  // return line to low
    }
  }

  LL_TIM_EnableCounter(TIM1);
  motor_dir = MOTOR_FORWARD;
}

static void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_clear_flag_tc(dma);

  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&port->gpio->BSRR);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&port_dma_buffer[index][0]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, DSHOT_DMA_BUFFER_SIZE);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  dshot_enable_dma_request(port->timer_channel);
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t j = 0; j < gpio_port_count; j++) {
      port_dma_buffer[j][i * 3 + 1] = 0; // clear middle bit
    }
    for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
      const uint32_t port = dshot_pins[motor].dshot_port;
      const uint32_t motor_high = (dshot_pins[motor].pin);
      const uint32_t motor_low = (dshot_pins[motor].pin << 16);

      const bool bit = dshot_packet[motor] & 0x8000;

      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      port_dma_buffer[port][i * 3 + 1] |= bit ? motor_high : motor_low;

      dshot_packet[motor] <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)port_dma_buffer, sizeof(port_dma_buffer));

#ifdef STM32F4
  while (spi_dma_is_ready(SPI_PORT1) == 0)
    __NOP();
#endif

  dshot_dma_phase = gpio_port_count;
  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_dma_setup_port(j);
  }
}

void motor_dshot_wait_for_ready() {
  while (dshot_dma_phase != 0)
    __NOP();
}

void dshot_dma_isr(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  dma_clear_flag_tc(dma);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  dshot_disable_dma_request(port->timer_channel);

  dshot_dma_phase--;
}
#endif