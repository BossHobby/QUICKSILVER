#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/failloop.h"
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
#define DSHOT_DMA_BUFFER_SIZE (3 * (16 + 2))

typedef struct {
  gpio_port_t *port;
  uint32_t pin;

  uint32_t dshot_port;
} dshot_pin_t;

typedef struct {
  gpio_port_t *gpio;

  uint32_t port_low;  // motor pins for BSRRL, for setting pins low
  uint32_t port_high; // motor pins for BSRRH, for setting pins high

  resource_tag_t timer_tag;

  dma_device_t dma_device;
  const dma_assigment_t *dma_ass;
} dshot_gpio_port_t;

extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern motor_direction_t motor_dir;

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

static uint8_t gpio_port_count = 0;
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];
static dshot_gpio_port_t gpio_ports[DSHOT_MAX_PORT_COUNT];

static volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

static const resource_tag_t timers[] = {
#ifndef STM32F411
    TIMER_TAG(TIMER8, TIMER_CH1),
    TIMER_TAG(TIMER8, TIMER_CH2),
    TIMER_TAG(TIMER8, TIMER_CH3),
    TIMER_TAG(TIMER8, TIMER_CH4),
#endif
    TIMER_TAG(TIMER1, TIMER_CH1),
    TIMER_TAG(TIMER1, TIMER_CH2),
    TIMER_TAG(TIMER1, TIMER_CH3),
    TIMER_TAG(TIMER1, TIMER_CH4),
};
static const uint32_t timer_count = sizeof(timers) / sizeof(resource_tag_t);

static const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev) {
  return &gpio_ports[dev - DMA_DEVICE_DSHOT_CH1];
}

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
      gpio_ports[i].dma_device = DMA_DEVICE_DSHOT_CH1 + i;
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
  for (uint8_t i = 0; i < timer_count; i++) {
    const resource_tag_t tag = timers[i];
    if (timer_alloc_tag(TIMER_USE_MOTOR_DSHOT, tag)) {
      port->timer_tag = tag;
    }
  }
  if (port->timer_tag == 0) {
    failloop(FAILLOOP_DMA);
  }

  const dma_assigment_t *dma = port->dma_ass = dma_alloc(port->dma_device, port->timer_tag);
  if (dma == NULL) {
    failloop(FAILLOOP_DMA);
  }

  const timer_channel_t ch = TIMER_TAG_CH(port->timer_tag);
  const timer_index_t tim = TIMER_TAG_TIM(port->timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  rcc_enable(def->rcc);

  // setup timer to 1/3 of the full bit time
  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(def->instance, &tim_init);
  LL_TIM_EnableARRPreload(def->instance);
  LL_TIM_DisableMasterSlaveMode(def->instance);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;
  LL_TIM_OC_Init(def->instance, ch, &tim_oc_init);
  LL_TIM_OC_EnablePreload(def->instance, ch);

  dma_enable_rcc(dma);
  LL_DMA_DeInit(dma->def->port, dma->def->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = dma->chan->request;
#else
  DMA_InitStructure.Channel = dma->chan->channel;
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
  LL_DMA_Init(dma->def->port, dma->def->stream_index, &DMA_InitStructure);

  interrupt_enable(dma->def->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma->def->port, dma->def->stream_index);

  LL_TIM_EnableCounter(def->instance);
}

void motor_dshot_init() {
  gpio_port_count = 0;

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_init_gpio_port(&gpio_ports[j]);

    // set all ports to low before the packet
    port_dma_buffer[j][0] = gpio_ports[j].port_low;
    port_dma_buffer[j][1] = gpio_ports[j].port_low;
    port_dma_buffer[j][2] = gpio_ports[j].port_low;

    for (uint8_t i = 0; i < 16; i++) {
      port_dma_buffer[j][(i + 1) * 3 + 0] = gpio_ports[j].port_high; // start bit
      port_dma_buffer[j][(i + 1) * 3 + 1] = 0;                       // actual bit, set below
      port_dma_buffer[j][(i + 1) * 3 + 2] = gpio_ports[j].port_low;  // return line to low
    }

    // set all ports to low after the packet
    port_dma_buffer[j][16 * 3 + 0] = gpio_ports[j].port_low;
    port_dma_buffer[j][16 * 3 + 1] = gpio_ports[j].port_low;
    port_dma_buffer[j][16 * 3 + 2] = gpio_ports[j].port_low;
  }

  motor_dir = MOTOR_FORWARD;
}

static void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_assigment_t *dma = port->dma_ass;

  dma_clear_flag_tc(dma->def);

  LL_DMA_SetPeriphAddress(dma->def->port, dma->def->stream_index, (uint32_t)&port->gpio->BSRR);
  LL_DMA_SetMemoryAddress(dma->def->port, dma->def->stream_index, (uint32_t)&port_dma_buffer[index][0]);
  LL_DMA_SetDataLength(dma->def->port, dma->def->stream_index, DSHOT_DMA_BUFFER_SIZE);

  LL_DMA_EnableStream(dma->def->port, dma->def->stream_index);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  for (uint8_t i = 0; i < 16; i++) {
    for (uint32_t j = 0; j < gpio_port_count; j++) {
      port_dma_buffer[j][(i + 1) * 3 + 1] = 0; // clear middle bit
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

void dshot_dma_isr(const dma_assigment_t *ass) {
  dma_clear_flag_tc(ass->def);
  LL_DMA_DisableStream(ass->def->port, ass->def->stream_index);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(ass->dev);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), false);

  dshot_dma_phase--;
}
#endif