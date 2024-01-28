#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"
#include "driver/spi.h"
#include "flight/control.h"

#ifdef USE_MOTOR_DSHOT

typedef struct {
  gpio_port_t *port;
  uint32_t pin;

  uint32_t set_mask;
  uint32_t reset_mask;

  uint32_t dshot_port;
  uint16_t *dma_input_buffer;
  uint32_t *dma_output_buffer;
} dshot_pin_t;

typedef struct {
  gpio_port_t *gpio;

  uint32_t set_mask;
  uint32_t reset_mask;

  uint32_t timer_channel;
  dma_device_t dma_device;
} dshot_gpio_port_t;

extern bool dshot_inverted;
extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern motor_direction_t motor_dir;

volatile uint32_t dshot_dma_phase = 0;       // 0: idle, 1 - (gpio_port_count + 1): handle port n
volatile uint32_t dshot_dma_input_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

static volatile bool dshot_output_phase = true;
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
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];
static volatile DMA_RAM uint32_t dma_output_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];
static volatile DMA_RAM uint16_t dma_input_buffer[DSHOT_MAX_PORT_COUNT][GCR_DMA_BUFFER_SIZE];

static const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev) {
  return &gpio_ports[dev - DMA_DEVICE_TIM1_CH1];
}

static void dshot_init_motor_pin_output(uint32_t index) {
  gpio_config_t config;
  config.mode = GPIO_OUTPUT;
  config.output = GPIO_PUSHPULL;
  config.drive = GPIO_DRIVE_HIGH;
  config.pull = GPIO_NO_PULL;
  gpio_pin_init(target.motor_pins[index], config);
}

static void dshot_init_motor_pin_input(uint32_t index) {
  gpio_config_t config;
  config.mode = GPIO_INPUT;
  config.output = GPIO_OPENDRAIN;
  config.drive = GPIO_DRIVE_HIGH;
  config.pull = GPIO_NO_PULL;
  gpio_pin_init(target.motor_pins[index], config);
}

static void dshot_init_motor_pin(uint32_t index) {
  dshot_pins[index].port = gpio_pin_defs[target.motor_pins[index]].port;
  dshot_pins[index].pin = gpio_pin_defs[target.motor_pins[index]].pin;
  dshot_pins[index].dshot_port = 0;

  const uint32_t motor_high = (dshot_pins[index].pin);
  const uint32_t motor_low = (dshot_pins[index].pin << 16);

  dshot_pins[index].set_mask = dshot_inverted ? motor_low : motor_high;
  dshot_pins[index].reset_mask = dshot_inverted ? motor_high : motor_low;

  dshot_init_motor_pin_output(index);
  LL_GPIO_ResetOutputPin(dshot_pins[index].port, dshot_pins[index].pin);

  for (uint8_t i = 0; i < DSHOT_MAX_PORT_COUNT; i++) {
    if (gpio_ports[i].gpio == dshot_pins[index].port || i == gpio_port_count) {
      // we already got a matching port in our array
      // or we reached the first empty spot
      gpio_ports[i].gpio = dshot_pins[index].port;
      gpio_ports[i].set_mask |= dshot_inverted ? (dshot_pins[index].pin << 16) : dshot_pins[index].pin;
      gpio_ports[i].reset_mask |= dshot_inverted ? dshot_pins[index].pin : (dshot_pins[index].pin << 16);

      dshot_pins[index].dshot_port = i;
      dshot_pins[index].dma_input_buffer = (uint16_t *)dma_input_buffer[i];
      dshot_pins[index].dma_output_buffer = (uint32_t *)dma_output_buffer[i];

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
  DMA_InitStructure.PeriphOrM2MSrcAddress = 0x0; // overwritten later
  DMA_InitStructure.MemoryOrM2MDstAddress = 0x0; // overwritten later
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
  tim_init.Autoreload = DSHOT_SYMBOL_FREQ;
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

    dma_output_buffer[j][0] = gpio_ports[j].reset_mask;
    dma_output_buffer[j][1] = gpio_ports[j].reset_mask;
    dma_output_buffer[j][2] = gpio_ports[j].reset_mask;

    for (uint32_t i = 3; i < DSHOT_DMA_BUFFER_SIZE; i += 3) {
      dma_output_buffer[j][i + 0] = gpio_ports[j].set_mask;
      dma_output_buffer[j][i + 1] = 0;
      dma_output_buffer[j][i + 2] = gpio_ports[j].reset_mask;
    }
  }

  LL_TIM_EnableCounter(TIM1);
  motor_dir = MOTOR_FORWARD;
}

static void dshot_dma_setup_output(uint32_t index) {
  const dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma->stream->PAR = (uint32_t)&port->gpio->BSRR;
  dma->stream->M0AR = (uint32_t)&dma_output_buffer[index][0];
  dma->stream->NDTR = DSHOT_DMA_BUFFER_SIZE;
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  dshot_enable_dma_request(port->timer_channel);
}

static void dshot_dma_setup_input(uint32_t index) {
  const dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma->stream->PAR = (uint32_t)&port->gpio->IDR;
  dma->stream->M0AR = (uint32_t)&dma_input_buffer[index][0];
  dma->stream->NDTR = GCR_DMA_BUFFER_SIZE;
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  dshot_enable_dma_request(port->timer_channel);
}

// make dshot dma packet, then fire
void dshot_dma_start() {
  if (dshot_inverted) {
    LL_TIM_SetAutoReload(TIM1, DSHOT_SYMBOL_FREQ);
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      state.gcr_values[i] = dshot_decode_gcr(dshot_pins[i].dma_input_buffer, dshot_pins[i].pin);
      dshot_init_motor_pin_output(i);
    }
  }

  for (uint8_t i = 0; i < 16; i++) {
    dma_output_buffer[0][(i + 1) * 3 + 1] = 0; // reset databit
    dma_output_buffer[1][(i + 1) * 3 + 1] = 0; // reset databit
    dma_output_buffer[2][(i + 1) * 3 + 1] = 0; // reset databit

    for (uint8_t motor = 0; motor < MOTOR_PIN_MAX; motor++) {
      const uint32_t port = dshot_pins[motor].dshot_port;

      // for 1 hold the line high for two timeunits
      // first timeunit is already applied
      const bool bit = dshot_packet[motor] & 0x8000;
      const uint32_t val = bit ? dshot_pins[motor].set_mask : dshot_pins[motor].reset_mask;
      dma_output_buffer[port][(i + 1) * 3 + 1] |= val;

      dshot_packet[motor] <<= 1;
    }
  }

  dma_prepare_tx_memory((void *)dma_output_buffer, sizeof(dma_output_buffer));
  dma_prepare_rx_memory((void *)dma_input_buffer, sizeof(dma_input_buffer));

#ifdef STM32F4
  while (spi_dma_is_ready(SPI_PORT1) == 0)
    __NOP();
#endif

  dshot_output_phase = true;
  dshot_dma_phase = gpio_port_count;
  for (uint32_t j = 0; j < gpio_port_count; j++) {
    dshot_dma_setup_output(j);
  }
}

void motor_dshot_wait_for_ready() {
  while (dshot_dma_phase != 0 || dshot_dma_input_phase != 0)
    __NOP();
}

void dshot_dma_isr(dma_device_t dev) {
  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  dma_clear_flag_tc(dev);

  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  LL_DMA_DisableStream(dma->port, dma->stream_index);
  dshot_disable_dma_request(port->timer_channel);

  if (dshot_output_phase) {
    dshot_dma_phase--;
    if (dshot_dma_phase == 0 && dshot_inverted) {
      LL_TIM_SetAutoReload(TIM1, GCR_SYMBOL_FREQ);
      for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
        dshot_init_motor_pin_input(i);
      }

      dshot_output_phase = false;
      dshot_dma_input_phase = gpio_port_count;
      for (uint32_t j = 0; j < gpio_port_count; j++) {
        dshot_dma_setup_input(j);
      }
    }
  } else {
    dshot_dma_input_phase--;
  }
}
#endif