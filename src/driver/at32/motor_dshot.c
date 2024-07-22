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

extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern dshot_pin_t dshot_pins[MOTOR_PIN_MAX];
extern volatile uint32_t dshot_dma_phase;

extern uint8_t dshot_gpio_port_count;
extern dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT];

extern volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

static const resource_tag_t timers[] = {
    TIMER_TAG(TIMER8, TIMER_CH1),
    TIMER_TAG(TIMER8, TIMER_CH2),
    TIMER_TAG(TIMER8, TIMER_CH3),
    TIMER_TAG(TIMER8, TIMER_CH4),
    TIMER_TAG(TIMER1, TIMER_CH1),
    TIMER_TAG(TIMER1, TIMER_CH2),
    TIMER_TAG(TIMER1, TIMER_CH3),
    TIMER_TAG(TIMER1, TIMER_CH4),
};
static const uint32_t timer_count = sizeof(timers) / sizeof(resource_tag_t);

extern const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev);

void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  for (uint8_t i = 0; i < timer_count; i++) {
    const resource_tag_t tag = timers[i];
    if (timer_alloc_tag(TIMER_USE_MOTOR_DSHOT, tag)) {
      port->timer_tag = tag;
      break;
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
  tmr_base_init(def->instance, DSHOT_SYMBOL_TIME, 0);
  tmr_clock_source_div_set(def->instance, TMR_CLOCK_DIV1);
  tmr_cnt_dir_set(def->instance, TMR_COUNT_UP);
  tmr_period_buffer_enable(def->instance, TRUE);

  tmr_output_config_type tim_oc_init;
  tmr_output_default_para_init(&tim_oc_init);
  tim_oc_init.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tim_oc_init.oc_idle_state = TRUE;
  tim_oc_init.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tim_oc_init.oc_output_state = TRUE;
  tmr_output_channel_config(def->instance, timer_channel_val(ch), &tim_oc_init);
  tmr_output_channel_buffer_enable(def->instance, timer_channel_val(ch), TRUE);

  dma_enable_rcc(dma);

  dma_reset(dma->def->stream);
  dmamux_init(dma->def->mux, dma->chan->request);

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
  dma_init(dma->def->stream, &init);
  dma_interrupt_enable(dma->def->stream, DMA_FDT_INT, TRUE);

  interrupt_enable(dma->def->irq, DMA_PRIORITY);

  tmr_counter_enable(def->instance, TRUE);
}

void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_assigment_t *dma = port->dma_ass;

  dma_clear_flag_tc(dma->def);

  dma->def->stream->paddr = (uint32_t)(&port->gpio->scr);
  dma->def->stream->maddr = (uint32_t)&port_dma_buffer[index][0];
  dma->def->stream->dtcnt = DSHOT_DMA_BUFFER_SIZE;

  dma_channel_enable(dma->def->stream, TRUE);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void motor_dshot_wait_for_ready() {
  while (dshot_dma_phase != 0)
    __NOP();
}

void dshot_dma_isr(const dma_assigment_t *ass) {
  dma_clear_flag_tc(ass->def);
  dma_channel_enable(ass->def->stream, FALSE);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(ass->dev);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), false);

  dshot_dma_phase--;
}
#endif