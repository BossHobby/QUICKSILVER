#pragma once

#include <stdbool.h>

#include "blackbox_device.h"

void blackbox_device_flash_init();
blackbox_device_result_t blackbox_device_flash_update();
void blackbox_device_flash_reset();
void blackbox_device_flash_write_header();
void blackbox_device_flash_flush();

uint32_t blackbox_device_flash_usage();
bool blackbox_device_flash_ready();

void blackbox_device_flash_read(const uint32_t file_index, const uint32_t offset, uint8_t *buffer, const uint32_t size);