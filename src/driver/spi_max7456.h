#pragma once

#include <stdint.h>

#include "driver/osd.h"

#define MAX7456_READ_FLAG 0x80

#define MAX7456_COLS 32
#define MAX7456_ROWS 16

#define VM0 0x00
#define VM1 0x01
#define VOS 0x03

#define DMM 0x04
#define DMAH 0x05
#define DMAL 0x06
#define DMDI 0x07

#define CMM 0x08
#define CMAH 0x09
#define CMAL 0x0A
#define CMDI 0x0B

#define OSDM 0x0C

#define RB0 0x10

#define STAT 0xA2 // Status register read address

#define DMDO 0XB0
#define CMDO 0xC0

#ifdef AIRBOT_OSD_PATCH // airbot ab7456 chip cant time blinks unless there is no incoming video signal
#define BLINK 0x01
#else
#define BLINK 0x11
#endif

#define INVERT 0x09
#define TEXT 0x01

#define SYSTEMXPOS 7
#define SYSTEMYPOS 7

#define YES 1
#define NO 0

#define VM0_R 0x80
#define OSDBL_R 0xEC
#define OSDBL_W 0x6C

void max7456_init();
bool max7456_is_ready();
void max7456_intro();

uint8_t max7456_clear_async();
osd_system_t max7456_check_system();

bool max7456_can_fit(uint8_t size);
bool max7456_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size);
bool max7456_flush();

void osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size);
void osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size);
