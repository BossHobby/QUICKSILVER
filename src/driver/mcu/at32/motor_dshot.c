#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"

#ifdef USE_MOTOR_DSHOT

void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];

  rcc_enable(tim->rcc);

  // setup timer to 1/3 of the full bit time
  tmr_base_init(tim->instance, DSHOT_SYMBOL_TIME, 0);
  tmr_clock_source_div_set(tim->instance, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(tim->instance, TMR_COUNT_UP);
  tmr_period_buffer_enable(tim->instance, TRUE);

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = TRUE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tim_oc_init.oc_output_state = TRUE;

  const uint32_t ch = timer_channel_val(TIMER_TAG_CH(port->timer_tag));
  tmr_output_channel_config(tim->instance, ch, &tim_oc_init);
  tmr_output_channel_buffer_enable(tim->instance, ch, TRUE);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  dma_enable_rcc(dma);

  dma_reset(dma->stream);
  dmamux_init(dma->mux, target.dma[port->dma_device].request);

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
  dma_init(dma->stream, &init);
  dma_interrupt_enable(dma->stream, DMA_FDT_INT, TRUE);

  interrupt_enable(dma->irq, DMA_PRIORITY);

  tmr_counter_enable(tim->instance, TRUE);
}

void dshot_dma_setup_output(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];

  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];
  tmr_period_value_set(tim->instance, DSHOT_SYMBOL_TIME);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  dma->stream->ctrl_bit.dtd = 1; // DMA_DIR_MEMORY_TO_PERIPHERAL
  dma->stream->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_WORD;
  dma->stream->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_WORD;
  dma->stream->paddr = (uint32_t)(&port->gpio->scr);
  dma->stream->maddr = (uint32_t)(&dshot_output_buffer[index][0]);
  dma->stream->dtcnt_bit.cnt = DSHOT_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->stream, TRUE);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void dshot_dma_setup_input(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];

  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];
  tmr_period_value_set(tim->instance, GCR_SYMBOL_TIME);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  dma->stream->ctrl_bit.dtd = 0; // DMA_DIR_PERIPHERAL_TO_MEMORY
  dma->stream->ctrl_bit.mwidth = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  dma->stream->ctrl_bit.pwidth = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  dma->stream->paddr = (uint32_t)(&port->gpio->idt);
  dma->stream->maddr = (uint32_t)(&dshot_input_buffer[index][0]);
  dma->stream->dtcnt_bit.cnt = GCR_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->stream, TRUE);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void dshot_dma_isr(const dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[dev].dma];

  dma_clear_flag_tc(dma);
  dma_channel_enable(dma->stream, FALSE);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), false);

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