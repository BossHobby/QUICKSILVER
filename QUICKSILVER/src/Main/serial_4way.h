/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 * Author: 4712
*/
#pragma once

#include "defines.h"


#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
#define USE_SERIAL_4WAY_BLHELI_BOOTLOADER
//#define USE_SERIAL_4WAY_SK_BOOTLOADER // not implemented in Silverware yet

#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include "serial_4way_impl.h"

#define imC2 0
#define imSIL_BLB 1
#define imATM_BLB 2
#define imSK 3
#define imARM_BLB 4

extern uint8_t selected_esc;

extern ioMem_t ioMem;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[2];
    uint16_t word;
} uint8_16_u;

typedef union __attribute__ ((packed)) {
    uint8_t bytes[4];
    uint16_t words[2];
    uint32_t dword;
} uint8_32_u;

//extern uint8_32_u DeviceInfo;

bool isMcuConnected(void);
uint8_t esc4wayInit(void);
struct serialPort_s;
void esc4wayProcess(void);
void esc4wayRelease(void);
#endif

