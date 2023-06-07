#include "driver/serial_soft.h"

extern volatile soft_serial_t soft_serial_ports[SERIAL_SOFT_COUNT];

#define PORT soft_serial_ports[port - SERIAL_SOFT_START]
#define TIMER timer_defs[TIMER_TAG_TIM(PORT.timer)]

extern void soft_serial_rx_update(serial_ports_t port);
extern void soft_serial_tx_update(serial_ports_t port);

void soft_serial_timer_start(serial_ports_t port) {
  LL_TIM_SetCounter(TIMER.instance, 0);

  LL_TIM_ClearFlag_UPDATE(TIMER.instance);
  LL_TIM_EnableIT_UPDATE(TIMER.instance);

  LL_TIM_EnableCounter(TIMER.instance);
}

void soft_serial_timer_stop(serial_ports_t port) {
  LL_TIM_DisableIT_UPDATE(TIMER.instance);
  LL_TIM_DisableCounter(TIMER.instance);
}

void soft_serial_timer_irq_handler() {
  for (uint8_t port = SERIAL_SOFT_START; port < SERIAL_SOFT_MAX; port++) {
    if (PORT.timer == RESOURCE_INVALID) {
      continue;
    }

    if (!LL_TIM_IsActiveFlag_UPDATE(TIMER.instance)) {
      continue;
    }

    LL_TIM_ClearFlag_UPDATE(TIMER.instance);
    soft_serial_tx_update(port);
    soft_serial_rx_update(port);
  }
}