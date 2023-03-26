#pragma once

#include <stdbool.h>

#include "blackbox_device.h"

void blackbox_device_sdcard_init();
blackbox_device_result_t blackbox_device_sdcard_update();
void blackbox_device_sdcard_reset();
void blackbox_device_sdcard_write_header();
void blackbox_device_sdcard_flush();

uint32_t blackbox_device_sdcard_usage();
bool blackbox_device_sdcard_ready();

void blackbox_device_sdcard_read_backbox(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);