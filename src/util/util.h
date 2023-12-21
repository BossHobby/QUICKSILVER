#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/time.h"

#define DEGTORAD 0.017453292f
#define RADTODEG 57.29577951f

#define M_PI_F 3.14159265358979323846f

#define MEMORY_ALIGN(offset, size) (((offset) + ((size)-1)) & -(size))

#define WHILE_TIMEOUT(condition, timeout) \
  for (uint32_t start = time_millis(); (condition) && (time_millis() - start) < (timeout);)

#define LOG2_8BIT(v) (8 - 90 / (((v) / 4 + 14) | 1) - 2 / ((v) / 2 + 1))
#define LOG2_16BIT(v) (8 * ((v) > 255) + LOG2_8BIT((v) >> 8 * ((v) > 255)))
#define LOG2_32BIT(v) (16 * ((v) > 65535L) + LOG2_16BIT((v) * 1L >> 16 * ((v) > 65535L)))

#define SWAP(x, y)      \
  {                     \
    typeof(x) temp = x; \
    x = y;              \
    y = temp;           \
  }

#define constrain(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
#define min(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define max(a, b) \
  ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

// this should be precalculated by the compiler when parameters are constant
//(1 - alpha. filtertime = 1 / filter-cutoff-frequency) as long as filtertime > sampleperiod
#define FILTERCALC(sampleperiod, filtertime) (1.0f - (6.0f * (float)(sampleperiod)) / (3.0f * (float)(sampleperiod) + (float)(filtertime)))

#define MHZ_TO_HZ(mhz) (mhz * 1000000)

#define MAKE_SEMVER(major, minor, patch) ((major << 16) | (minor << 8) | patch)

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

float atan2approx(float y, float x);
int ipow(int base, int exp);
float fastsin(float x);
float fastcos(float x);

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2);
int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2);

uint32_t get_chip_uid();
