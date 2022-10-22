#include "project.h"

#if defined(RX_FLYSKY)
#include "drv_time.h"
#include "drv_gpio.h"
#include "rx_flysky.h"

//------------------------------------------------------------------------------
// This file contains hardware related functions and is kept separate
// from the rest of the RX code so that the RX code can be developed/used/tested
// with other projects/frameworks

//------------------------------------------------------------------------------
// If 'init' is true, this records state info about the bind button pin so we can
// later debounce and detect state changes. Otherwise it checks the state
// (performing a debounce check) and returns true if button was depressed
bool flysky_check_bind_button(bool init) {
  bool curr_state = gpio_pin_read(RX_BIND_PIN);
  if (init) {
    flysky.bind_pin_debounce_state = flysky.bind_pin_current_state = curr_state;
    return curr_state;
  }

  // If state has toggled then reload debounce timer
  if (flysky.bind_pin_current_state != curr_state) {
    flysky.bind_pin_current_state = curr_state;
    flysky.bind_pin_time = time_millis() + 50;  
  }
  else if (flysky.bind_pin_time) {  
    // otherwise if button is being debounced we check if it has
    // been stable in this state for the requested debounce time
    if (time_millis() >= flysky.bind_pin_time) {
      flysky.bind_pin_time = 0;
      if (flysky.bind_pin_debounce_state != flysky.bind_pin_current_state) {
        // Debounce complete, the logical state of the button has toggled
        // If it has toggled to low state (button pressed) then bind has been requested
        if (!flysky.bind_pin_current_state) {
          return true;
        }
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
static void blink_rx_led(uint32_t blink_interval_ms) {
  static uint32_t last_time;
  uint32_t now = time_millis();
  if (now >= (last_time + blink_interval_ms)) {
    last_time = now;
    gpio_pin_toggle(RX_LED_PIN);
  }
}

//------------------------------------------------------------------------------
// If failsafe the then slow blink, otherwise keep led steady (off)
void flysky_update_rx_led(bool failsafe) {
  if (failsafe) {
    blink_rx_led(1000);
  }
  else {
    // RX link is active, turn off RX led
    // Note: If you'd prefer to have it steady on, then change
    // this to use gpio_pin_set()
    gpio_pin_reset(RX_LED_PIN);
  }
}

//------------------------------------------------------------------------------
void flysky_fast_blink_rx_led() {
  blink_rx_led(250);
}

#endif
