#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"

#ifdef USE_MOTOR_DSHOT

extern volatile uint32_t dshot_dma_phase;
extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern dshot_pin_t dshot_pins[MOTOR_PIN_MAX];

extern motor_direction_t motor_dir;

extern uint8_t dshot_gpio_port_count;
dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT] = {
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

extern volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

extern void dshot_init_motor_pin(uint32_t index);
extern const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev);

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
  dshot_gpio_port_count = 0;

  rcc_enable(RCC_ENCODE(TMR1));

  // setup timer to 1/3 of the full bit time
  tmr_base_init(TMR1, DSHOT_SYMBOL_TIME, 0);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR1, TRUE);

  for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
    dshot_init_motor_pin(i);
  }

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_init_gpio_port(&dshot_gpio_ports[j]);

    for (uint8_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      port_dma_buffer[j][i * 3 + 0] = dshot_gpio_ports[j].port_high; // start bit
      port_dma_buffer[j][i * 3 + 1] = 0;                             // actual bit, set below
      port_dma_buffer[j][i * 3 + 2] = dshot_gpio_ports[j].port_low;  // return line to low
    }
  }

  tmr_counter_enable(TMR1, TRUE);
  motor_dir = MOTOR_FORWARD;
}

void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_clear_flag_tc(dma);

  dma->channel->paddr = (uint32_t)(&port->gpio->scr);
  dma->channel->maddr = (uint32_t)&port_dma_buffer[index][0];
  dma->channel->dtcnt = DSHOT_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->channel, TRUE);
  dshot_enable_dma_request(port->timer_channel, TRUE);
}

void motor_dshot_wait_for_ready() {
  while (dshot_dma_phase != 0)
    __NOP();
}

void dshot_dma_isr(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  dma_clear_flag_tc(dma);
  dma_channel_enable(dma->channel, FALSE);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  dshot_enable_dma_request(port->timer_channel, FALSE);

  dshot_dma_phase--;
}
#endif