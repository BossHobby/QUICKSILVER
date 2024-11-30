#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"

#ifdef USE_MOTOR_DSHOT

static void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  dma_enable_rcc(port->dma_device);

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = TRUE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tim_oc_init.oc_output_state = TRUE;
  tmr_output_channel_config(TMR1, timer_channel_val(port->timer_channel), &tim_oc_init);
  tmr_output_channel_buffer_enable(TMR1, timer_channel_val(port->timer_channel), TRUE);

  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_reset(dma->channel);
  dmamux_init(dma->mux, dma->request);

  dma_init_type init;
  init.peripheral_base_addr = 0x0; // overwritten later
  init.memory_base_addr = 0x0;     // overwritten later
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

void dshot_init_timer() {
  rcc_enable(RCC_ENCODE(TMR1));

  // setup timer to 1/3 of the full bit time
  tmr_base_init(TMR1, DSHOT_SYMBOL_TIME, 0);
  tmr_clock_source_div_set(TMR1, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
  tmr_period_buffer_enable(TMR1, TRUE);

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_init_gpio_port(&dshot_gpio_ports[j]);

    for (uint8_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      dshot_output_buffer[j][i * 3 + 0] = dshot_gpio_ports[j].set_mask;   // start bit
      dshot_output_buffer[j][i * 3 + 1] = 0;                              // actual bit, set below
      dshot_output_buffer[j][i * 3 + 2] = dshot_gpio_ports[j].reset_mask; // return line to low
    }
  }

  tmr_counter_enable(TMR1, TRUE);
}

void dshot_dma_setup_output(uint32_t index) {
  tmr_period_value_set(TMR1, DSHOT_SYMBOL_TIME);

  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma->channel->ctrl_bit.dtd = 1; // DMA_DIR_MEMORY_TO_PERIPHERAL
  dma->channel->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_WORD;
  dma->channel->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  dma->channel->paddr = (uint32_t)(&port->gpio->scr);
  dma->channel->maddr = (uint32_t)(&dshot_output_buffer[index][0]);
  dma->channel->dtcnt_bit.cnt = DSHOT_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->channel, TRUE);
  timer_enable_dma_request(TIMER1, port->timer_channel, true);
}

void dshot_dma_setup_input(uint32_t index) {
  tmr_period_value_set(TMR1, GCR_SYMBOL_TIME);

  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma->channel->ctrl_bit.dtd = 0; // DMA_DIR_PERIPHERAL_TO_MEMORY
  dma->channel->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma->channel->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma->channel->paddr = (uint32_t)(&port->gpio->idt);
  dma->channel->maddr = (uint32_t)(&dshot_input_buffer[index][0]);
  dma->channel->dtcnt_bit.cnt = GCR_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->channel, TRUE);
  timer_enable_dma_request(TIMER1, port->timer_channel, true);
}

void dshot_dma_isr(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  dma_clear_flag_tc(dma);
  dma_channel_enable(dma->channel, FALSE);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  timer_enable_dma_request(TIMER1, port->timer_channel, false);

  dshot_phase--;

  if (profile.motor.dshot_telemetry && dshot_phase == dshot_gpio_port_count) {
    // output phase done, lets swap to input
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      dshot_gpio_init_input(target.motor_pins[i]);
    }
    for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
      dshot_dma_setup_input(j);
    }
  }
}
#endif