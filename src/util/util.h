#pragma once

#include <stddef.h>
#include <stdint.h>

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define M_PI_F 3.14159265358979323846f

#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define min(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define max(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

float mapf(float x, float in_min, float in_max, float out_min, float out_max);
void limitf(float *input, const float limit);
float constrainf(const float in, const float min, const float max);

float atan2approx(float y, float x);
float Q_rsqrt(float number);
int ipow(int base, int exp);
float fastsin(float x);
float fastcos(float x);

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2);
int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2);

uint32_t get_chip_uid();
