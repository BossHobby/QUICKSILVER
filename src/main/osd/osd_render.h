#pragma once

#include <stdint.h>

void osd_init();
void osd_display();
void osd_clear();
uint8_t osd_decode(uint32_t element, uint8_t status);
