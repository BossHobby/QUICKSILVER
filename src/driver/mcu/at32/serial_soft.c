#include "driver/serial_soft.h"

extern volatile soft_serial_t soft_serial_ports[SERIAL_SOFT_COUNT];

#define PORT soft_serial_ports[port - SERIAL_SOFT_START]
#define TIMER timer_defs[TIMER_TAG_TIM(PORT.timer)]

extern void soft_serial_rx_update(serial_ports_t port);
extern void soft_serial_tx_update(serial_ports_t port);

void soft_serial_timer_start(serial_ports_t port) {
  tmr_counter_value_set(TIMER.instance, 0);

  tmr_flag_clear(TIMER.instance, TMR_OVF_FLAG);
  tmr_interrupt_enable(TIMER.instance, TMR_OVF_INT, TRUE);

  tmr_counter_enable(TIMER.instance, TRUE);
}

void soft_serial_timer_stop(serial_ports_t port) {
  tmr_interrupt_enable(TIMER.instance, TMR_OVF_INT, FALSE);
  tmr_counter_enable(TIMER.instance, FALSE);
}

void soft_serial_timer_irq_handler() {
  for (uint8_t port = SERIAL_SOFT_START; port < SERIAL_SOFT_MAX; port++) {
    if (PORT.timer == RESOURCE_INVALID) {
      continue;
    }

    if (!tmr_flag_get(TIMER.instance, TMR_OVF_FLAG)) {
      continue;
    }

    tmr_flag_clear(TIMER.instance, TMR_OVF_FLAG);

    soft_serial_tx_update(port);
    soft_serial_rx_update(port);
  }
}