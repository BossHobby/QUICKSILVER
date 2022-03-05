#pragma once

#include <stdint.h>

#include "drv_osd.h"

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

#define MAXCOLUMNS 32
#define MAXROWS 16

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

void max7456_txn_start(uint8_t attr, uint8_t x, uint8_t y);
void max7456_txn_write_char(const char val);
void max7456_txn_write_data(const uint8_t *buffer, uint8_t size);
void max7456_txn_submit(osd_transaction_t *txn);

void osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size);
void osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size);
