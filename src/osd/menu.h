#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "osd/render.h"
#include "util/vector.h"

// immediate mode ui for qs

typedef enum {
  OSD_AUTO = -1,
  OSD_END = -2,
} osd_layout_t;

void osd_menu_start();

void osd_menu_header(const char *text);
void osd_menu_label(int8_t x, int8_t y, const char *text);
bool osd_menu_label_start(int8_t x, int8_t y);
void osd_menu_highlight(int8_t x, int8_t y, const char *text);
bool osd_menu_button(int8_t x, int8_t y, const char *text);

int32_t osd_menu_adjust_int(int32_t val, const int32_t delta, const int32_t min, const int32_t max);
float osd_menu_adjust_float(float val, const float delta, const float min, const float max);
vec3_t osd_menu_adjust_vec3(vec3_t val, const float delta, const float min, const float max);
void osd_menu_adjust_str(char *str);

void osd_menu_scroll_start(uint8_t x, uint8_t y, uint8_t max);
void osd_menu_scroll_finish(uint8_t x);

void osd_menu_select(int8_t x, int8_t y, const char *text);
void osd_menu_select_save_and_exit(int8_t x);
void osd_menu_select_exit(int8_t x);
void osd_menu_select_screen(int8_t x, int8_t y, const char *text, osd_screens_t screen);
bool osd_menu_select_enum(int8_t x, int8_t y, const uint8_t val, const char **labels);
bool osd_menu_select_int(int8_t x, int8_t y, const int32_t val, uint8_t width);
bool osd_menu_select_str(int8_t x, int8_t y, const char *str);
bool osd_menu_select_float(int8_t x, int8_t y, const float val, uint8_t width, uint8_t precision);
bool osd_menu_select_vec3(int8_t x, int8_t y, const vec3_t val, uint8_t width, uint8_t precision);

void osd_menu_select_enum_adjust(int8_t x, int8_t y, const char *text, uint8_t select_x, uint8_t *val, const char **labels, const int32_t min, const int32_t max);

bool osd_menu_finish();
