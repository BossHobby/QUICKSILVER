#pragma once

#include <stdint.h>

void osd_init(void);
void osd_display(void);
void osd_clear(void);
uint8_t osd_decode(uint32_t element, uint8_t status);
