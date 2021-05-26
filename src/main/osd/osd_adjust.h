#pragma once

#include <stdint.h>

#include "util/vector.h"

#define STORE_VALUE 1
#define RETURN_VALUE 0
#define BF_PIDS 0
#define SW_RATES 1
#define ROUNDED 2

void osd_submenu_select(uint8_t *pointer, uint8_t rows, const uint8_t next_menu[]);
void osd_select_menu_item(uint8_t rows, const uint8_t menu_map[], uint8_t main_menu);
uint8_t last_cursor_array_stuffer(uint8_t cursor, uint8_t add_new);

void osd_encoded_adjust(uint32_t *pointer, uint8_t rows, uint8_t columns, uint8_t status);
void osd_encoded_adjust_callsign(void);

const char *get_rssi_source_status(void);
const char *get_aux_status(int input);
const char *get_vtx_status(int input);
vec3_t *get_pid_term(uint8_t term);
vec3_t *get_sw_rate_term(uint8_t term);
vec3_t *get_bf_rate_term(uint8_t term);
vec3_t *get_stick_profile_term(uint8_t term);

void osd_vector_adjust(vec3_t *pointer, uint8_t rows, uint8_t columns, uint8_t special_case, const float adjust_limit[rows * columns][2]);
void osd_float_adjust(float *pointer[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount);
void osd_enum_adjust(uint8_t *pointer[], uint8_t rows, const uint8_t increase_limit[]);
void osd_mixed_data_adjust(float *pointer[], uint8_t *pointer2[], uint8_t rows, uint8_t columns, const float adjust_limit[rows * columns][2], float adjust_amount, const uint8_t reboot_request[rows * columns]);
void populate_vtx_buffer_once(void);
