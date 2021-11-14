#pragma once

#include "project.h"
#include "stdint.h"

#define MAX7456_READ_FLAG 0x80

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

#define STAT 0xA2 //Status register read address

#define DMDO 0XB0
#define CMDO 0xC0

#ifdef AIRBOT_OSD_PATCH //airbot ab7456 chip cant time blinks unless there is no incoming video signal
#define BLINK 0x01
#else
#define BLINK 0x11
#endif

#define INVERT 0x09
#define TEXT 0x01
#define MAXROWS 16

#define SYSTEMXPOS 7
#define SYSTEMYPOS 7

// do not change defines below this line
#define PAL 1
#define NTSC 0
#define NONE 2

#define YES 1
#define NO 0

#define VM0_R 0x80
#define OSDBL_R 0xEC
#define OSDBL_W 0x6C

void spi_max7456_init(void);
void max7456_init(void);

void osd_intro(void);
void osd_clear(void);
uint8_t osd_runtime_screen_clear(void);
void osd_checksystem(void);
void osd_print(const char *buffer, uint8_t dmm_attribute, uint8_t x, uint8_t y);
void osd_print_data(uint8_t *buffer, uint8_t length, uint8_t dmm_attribute, uint8_t x, uint8_t y);
void osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size);
void osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size);

void fast_fprint(uint8_t *str, uint8_t length, float v, uint8_t precision);
