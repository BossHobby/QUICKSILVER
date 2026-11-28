#include "driver/timer.h"

#include "core/profile.h"
#include "driver/gpio.h"

#define TIMER_ASSIGMENT_MAX 32

timer_assigment_t timer_assigments[TIMER_ASSIGMENT_MAX] = {};

static void timer_alloc_dma(timer_use_t use, const target_dma_t *entry) {
  if (entry->dma == DMA_STREAM_INVALID || RESOURCE_TAG_TYPE(entry->tag) != RESOURCE_TIM) {
    return;
  }

  timer_alloc_tag(use, entry->tag);
}

void timer_alloc_init() {
  // motor_dshot_init() gives each GPIO port with a DShot output the next
  // DSHOT_CHn stream, whichever output slots the port serves.
  const gpio_port_t *dshot_ports[DMA_DEVICE_DSHOT_CH3 - DMA_DEVICE_DSHOT_CH1 + 1] = {};
  uint32_t dshot_port_count = 0;
  for (uint32_t i = 0; i < DSHOT_OUTPUT_MAX; i++) {
    const gpio_pins_t pin = target.outputs[i].pin;
    if (pin == PIN_NONE || (target.outputs[i].caps & OUTPUT_CAP_DSHOT) == 0 ||
        !profile_output_slot_uses_protocol(i, OUTPUT_PROTOCOL_DSHOT)) {
      continue;
    }

    const gpio_port_t *port = gpio_pin_defs[pin].port;
    uint32_t j = 0;
    while (j < dshot_port_count && dshot_ports[j] != port) {
      j++;
    }
    if (j == dshot_port_count && dshot_port_count < DMA_DEVICE_DSHOT_CH3 - DMA_DEVICE_DSHOT_CH1 + 1) {
      dshot_ports[dshot_port_count++] = port;
    }
  }

  for (uint32_t i = 0; i < dshot_port_count; i++) {
    timer_alloc_dma(TIMER_USE_MOTOR_DSHOT, &target.dma[DMA_DEVICE_DSHOT_CH1 + i]);
  }

  timer_alloc_dma(TIMER_USE_RGB_LED, &target.dma[DMA_DEVICE_RGB]);
}

bool timer_alloc_tag(timer_use_t use, resource_tag_t tag) {
  for (uint32_t i = 0; i < TIMER_ASSIGMENT_MAX; i++) {
    timer_assigment_t *ass = &timer_assigments[i];
    if (ass->use == TIMER_USE_FREE) {
      // found free spot
      timer_assigments[i].use = use;
      timer_assigments[i].tag = tag;
      return true;
    }
    if (TIMER_TAG_TIM(ass->tag) != TIMER_TAG_TIM(tag)) {
      continue;
    }
    if (ass->use != use) {
      // timer reserved and use does not match
      return false;
    }
    if ((TIMER_TAG_CH(ass->tag) & TIMER_TAG_CH(tag)) != 0) {
      // use matches but all timer channels are used up
      return false;
    }
    // add channel to existing assigment
    ass->tag = TIMER_TAG(TIMER_TAG_TIM(ass->tag), TIMER_TAG_CH(ass->tag) | TIMER_TAG_CH(tag));
    return true;
  }

  // we ran out of assigments
  return false;
}

resource_tag_t timer_alloc(timer_use_t use) {
  for (uint8_t i = TIMER_INVALID + 1; i < TIMER_MAX; i++) {
    const resource_tag_t tag = TIMER_TAG(i, TIMER_CH_ALL);
    if (timer_alloc_tag(use, tag)) {
      return tag;
    }
  }
  return TIMER_TAG(TIMER_INVALID, TIMER_CH_INVALID);
}
