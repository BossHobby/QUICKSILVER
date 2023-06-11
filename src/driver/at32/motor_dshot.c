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

  uint32_t timer_channel;
  dma_device_t dma_device;
} dshot_gpio_port_t;

extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern uint32_t pwm_failsafe_time;
extern motor_direction_t motor_dir;

volatile uint32_t dshot_dma_phase = 0; // 0: idle, 1 - (gpio_port_count + 1): handle port n

static uint8_t gpio_port_count = 0;
static dshot_gpio_port_t gpio_ports[DSHOT_MAX_PORT_COUNT] = {
    {
        .timer_channel = TMR_SELECT_CHANNEL_1,
        .dma_device = DMA_DEVICE_TIM1_CH1,
    },
    {
        .timer_channel = TMR_SELECT_CHANNEL_3,
        .dma_device = DMA_DEVICE_TIM1_CH3,
    },
    {
        .timer_channel = TMR_SELECT_CHANNEL_4,
        .dma_device = DMA_DEVICE_TIM1_CH4,
    },
};
static volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];
static dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

static void dshot_init_motor_pin(uint32_t index) {
  dshot_pins[index].port = gpio_pin_defs[target.motor_pins[index]].port;
  dshot_pins[index].pin = gpio_pin_defs[target.motor_pins[index]].pin;
  dshot_pins[index].dshot_port = 0;

  gpio_init_type init;
  init.gpio_mode = GPIO_MODE_OUTPUT;
  init.gpio_drive_strength = GPIO_DRIVE_HIGH;
  init.gpio_out_type = GPIO_PUSHPULL;
  init.gpio_pull = GPIO_PULL_NONE;
  init.gpio_pins = dshot_pins[index].pin;
  gpio_init(dshot_pins[index].port, &init);
  gpio_bits_reset(dshot_pins[index].port, dshot_pins[index].pin);

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
  dma_enable_rcc(port->dma_device);

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = TRUE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tim_oc_init.oc_output_state = TRUE;
  tmr_output_channel_config(TMR1, port->timer_channel, &tim_oc_init);
  tmr_output_channel_buffer_enable(TMR1, port->timer_channel, TRUE);

  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_reset(dma->channel);
  dmamux_init(dma->mux, dma->request);

  dma_init_type init;
  init.peripheral_base_addr = (uint32_t)(&port->gpio->scr);
  init.memory_base_addr = (uint32_t)port_dma_buffer[0];
  init.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  init.buffer_size = DSHOT_DMA_BUFFER_SIZE;
  init.peripheral_inc_enable = FALSE;
  init.memory_inc_enable = TRUE;
  init.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  init.memory_data_width = DMA_MEMORY_DATA_WIDTH_WORD;
  init.loop_mode_enable = FALSE;
  init.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init(dma->channel, &init);
  dma_interrupt_enable(dma->channel, DMA_FDT_INT, TRUE);

  interrupt_enable(dma->irq, DMA_PRIORITY);
}

static void dshot_enable_dma_request(uint32_t timer_channel, confirm_state new_state) {
  switch (timer_channel) {
  case TMR_SELECT_CHANNEL_1:
    tmr_dma_request_enable(TMR1, TMR_C1_DMA_REQUEST, new_state);
    break;
  case TMR_SELECT_CHANNEL_3:
    tmr_dma_request_enable(TMR1, TMR_C3_DMA_REQUEST, new_state);
    break;
  case TMR_SELECT_CHANNEL_4:
    tmr_dma_request_enable(TMR1, TMR_C4_DMA_REQUEST, new_state);
    break;
  default:
    break;
  }
}

void motor_dshot_init() {
  gpio_port_count = 0;

  rcc_enable(RCC_ENCODE(TMR1));

  // setup timer to 1/3 of the full bit time
  tmr_base_init(TMR1, DSHOT_SYMBOL_TIME, 0);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR1, TRUE);

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

  tmr_counter_enable(TMR1, TRUE);

  // set failsafetime so signal is off at start
  pwm_failsafe_time = time_micros() - 100000;
  motor_dir = MOTOR_FORWARD;
}

static void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_clear_flag_tc(port->dma_device);

  dma->channel->paddr = (uint32_t)(&port->gpio->scr);
  dma->channel->maddr = (uint32_t)&port_dma_buffer[index][0];
  dma->channel->dtcnt = DSHOT_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->channel, TRUE);
  dshot_enable_dma_request(port->timer_channel, TRUE);
}

// make dshot dma packet, then fire
void dshot_dma_start() {
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

void dshot_dma_isr(dma_device_t dev) {
  for (uint32_t j = 0; j < gpio_port_count; j++) {
    const dshot_gpio_port_t *port = &gpio_ports[j];
    if (port->dma_device != dev) {
      continue;
    }

    dma_clear_flag_tc(port->dma_device);

    const dma_stream_def_t *dma = &dma_stream_defs[dev];
    dma_channel_enable(dma->channel, FALSE);
    dshot_enable_dma_request(port->timer_channel, FALSE);

    dshot_dma_phase--;
    break;
  }
}
#endif