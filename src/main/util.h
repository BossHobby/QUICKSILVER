#pragma once

#include <stddef.h>
#include <stdint.h>

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void limitf(float *input, const float limit);
float constrainf(const float in, const float min, const float max);

uint32_t min_uint32(uint32_t a, uint32_t b);

int round_num(float num);
float round_dec2(float num);

float atan2approx(float y, float x);
float Q_rsqrt(float number);
int ipow(int base, int exp);
float fastsin(float x);
float fastcos(float x);

void limit180(float *);

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2);
int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2);

void reset_looptime();