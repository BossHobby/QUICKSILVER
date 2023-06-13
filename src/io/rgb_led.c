#include "io/rgb_led.h"

#include "core/project.h"
#include "driver/rgb_led.h"

static const uint32_t green = RGB(98, 168, 52);

void rgb_led_update() {
#if defined(USE_RGB_LED)
  if (rgb_led_busy()) {
    return;
  }

  rgb_led_set_value(green, 2);
  rgb_led_send();
#endif
}
