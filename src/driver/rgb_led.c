#include "driver/rgb_led.h"

#include "core/project.h"

volatile bool rgb_dma_busy = false;
resource_tag_t rgb_timer_tag = 0;

#if !defined(USE_RGB_LED)
void rgb_led_init() {
}

void rgb_led_send(uint32_t count) {
  (void)count;
}
#endif

bool rgb_led_busy() {
  if (target.rgb_led == PIN_NONE || rgb_timer_tag == 0)
    return true;
  return rgb_dma_busy;
}