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

#include <stdbool.h>
#include <stdint.h>

extern uint8_t selected_esc;

bool is_esc_high(uint8_t esc);
bool is_esc_low(uint8_t esc);
void set_esc_high(uint8_t esc);
void set_esc_low(uint8_t esc);
void set_esc_input(uint8_t esc);
void set_esc_output(uint8_t esc);

#define ESC_IS_HI is_esc_high(selected_esc)
#define ESC_IS_LO is_esc_low(selected_esc)
#define ESC_SET_HI set_esc_high(selected_esc)
#define ESC_SET_LO set_esc_low(selected_esc)
#define ESC_INPUT set_esc_input(selected_esc)
#define ESC_OUTPUT set_esc_output(selected_esc)

bool is_mcu_connected();

typedef struct ioMem_s {
  uint8_t D_NUM_BYTES;
  uint8_t D_FLASH_ADDR_H;
  uint8_t D_FLASH_ADDR_L;
  uint8_t *D_PTR_I;
} ioMem_t;
