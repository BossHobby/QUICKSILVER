#include "io/pinio.h"

#include "control/control.h"
#include "core/profile.h"
#include "driver/gpio.h"
#include "rx/rx.h"

struct pinio_output_t {
  gpio_pins_t pin;
  bool invert;
  bool initialized;
};

// Hardware ownership is fixed at boot. USB may edit target metadata before
// rebooting; those edits must not redirect writes to an uninitialized pin.
static pinio_output_t outputs[PINIO_MAX];
static_assert(AUX_PINIO_4 - AUX_PINIO_1 + 1 == PINIO_MAX);
static_assert(AUX_FUNCTION_MAX <= 32);

static void pinio_write(const pinio_output_t &output, bool active) {
  if (active != output.invert) {
    gpio_pin_set(output.pin);
  } else {
    gpio_pin_reset(output.pin);
  }
}

static void pinio_init_output(uint8_t index, bool active) {
  auto &output = outputs[index];
  // Preload the latch before enabling output mode, including active-low pins.
  pinio_write(output, active);
  gpio_config_t config = gpio_config_default();
  config.drive = GPIO_DRIVE_HIGH;
  gpio_pin_init(output.pin, config);
  output.initialized = true;
}

void pinio_init() {
  for (uint8_t i = 0; i < PINIO_MAX; i++) {
    outputs[i] = {target.pinio[i].pin, target.pinio[i].invert, false};
    const auto &output = outputs[i];
    if (output.pin == PIN_NONE || output.pin >= PINS_MAX)
      continue;
    // Keep SWD available until the output is first switched on with the
    // receiver ready, as on the former FPV pins.
    if (output.pin == PIN_A13 || output.pin == PIN_A14)
      continue;
    const auto channel = profile.receiver.aux[AUX_PINIO_1 + i].channel;
    pinio_init_output(i, channel == RX_CHANNEL_ON);
  }
}

TickType_t pinio_update() {
  bool configured = false;
  for (uint8_t i = 0; i < PINIO_MAX; i++) {
    const auto &output = outputs[i];
    if (output.pin == PIN_NONE || output.pin >= PINS_MAX)
      continue;
    configured = true;
    const bool active = rx_aux_on(AUX_PINIO_1 + i);
    if (output.initialized) {
      pinio_write(output, active);
    } else if (active && flags.rx_mode == RXMODE_NORMAL) {
      pinio_init_output(i, active);
    }
  }
  return configured ? pdMS_TO_TICKS(10) : portMAX_DELAY;
}
