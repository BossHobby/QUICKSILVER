#include "driver/rgb_led.h"

#include "core/project.h"

volatile bool rgb_dma_busy = false;
resource_tag_t rgb_timer_tag = 0;

bool rgb_led_busy() {
  if (target.rgb_led == PIN_NONE)
    return true;
  return rgb_dma_busy;
}