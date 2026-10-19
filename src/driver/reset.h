#pragma once

#include <stdint.h>

#define RESET_FIRMWARE_MAGIC 0xCAFEBABE
#define RESET_BOOTLOADER_MAGIC 0xDEADBEEF
// The low byte carries the failloop code across the reset.
#define RESET_FAILLOOP_MAGIC 0xFA110000
#define RESET_FAILLOOP_CODE_MASK 0xFF

// Failloop code handed over by system_reset_to_failloop(), set by the boot
// check before main(); zero on a normal boot.
extern uint8_t system_reset_failloop;

void system_reset();
void system_reset_to_bootloader();
void system_reset_to_failloop(uint8_t code);
