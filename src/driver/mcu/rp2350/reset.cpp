#include "driver/reset.h"

#include <hardware/structs/watchdog.h>
#include <pico/bootrom.h>
#include <pico/runtime_init.h>

#include "core/project.h"

// Watchdog scratch 0 to 3 are free for the application and keep their value
// across a system reset; the bootrom owns 4 to 7.
#define RESET_SCRATCH watchdog_hw->scratch[0]

void system_reset_to_bootloader() {
  // The bootrom's USB mass storage and PICOBOOT interfaces take the UF2.
  rom_reset_usb_boot(0, 0);
}

void system_reset_to_failloop(uint8_t code) {
  RESET_SCRATCH = RESET_FAILLOOP_MAGIC | code;
  system_reset();
}

// Runs before main() from the SDK's runtime initialisation.
static void reset_check_failloop() {
  const uint32_t magic = RESET_SCRATCH;
  if ((magic & ~RESET_FAILLOOP_CODE_MASK) == RESET_FAILLOOP_MAGIC)
    system_reset_failloop = magic & RESET_FAILLOOP_CODE_MASK;
  RESET_SCRATCH = RESET_FIRMWARE_MAGIC;
}
PICO_RUNTIME_INIT_FUNC_RUNTIME(reset_check_failloop, "11000");
