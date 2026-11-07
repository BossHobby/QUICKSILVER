#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/osd/osd.h"

// Analog OSD the MCU draws into the camera signal itself (target.video_osd),
// with a MAX7456-format font kept in MCU flash.

bool video_osd_init();
bool video_osd_is_ready();
void video_osd_intro();

bool video_osd_clear_async();
osd_system_t video_osd_check_system();

uint32_t video_osd_can_fit();
bool video_osd_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool video_osd_flush();

void video_osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size);
void video_osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size);
