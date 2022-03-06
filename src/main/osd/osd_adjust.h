#pragma once

#include <stdint.h>

#include "util/vector.h"

#define BF_PIDS 0
#define SW_RATES 1
#define ROUNDED 2

uint8_t osd_push_cursor();
uint8_t osd_pop_cursor();

void osd_encoded_adjust(uint32_t *pointer, uint8_t rows, uint8_t columns, uint8_t status);
void osd_encoded_adjust_callsign();

void osd_save_exit();

const char *get_rssi_source_status(uint8_t data_to_print);
const char *get_vtx_status(int input);

void osd_float_adjust(float *pointer[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount);
void osd_enum_adjust(uint8_t *pointer[], uint8_t rows, const uint8_t increase_limit[]);
void populate_vtx_buffer_once();
