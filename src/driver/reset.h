#pragma once

#define RESET_FIRMWARE_MAGIC 0xCAFEBABE
#define RESET_BOOTLOADER_MAGIC 0xDEADBEEF

void system_reset();
void system_reset_to_bootloader();
