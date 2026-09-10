#pragma once

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

#include "driver/time.h"

constexpr float DEGTORAD = 0.017453292f;
constexpr float RADTODEG = 57.29577951f;
constexpr float M_PI_F = 3.14159265358979323846f;

#define MEMORY_ALIGN(offset, size) (((offset) + ((size) - 1)) & -(size))

// Force inline for performance-critical functions
#define FORCE_INLINE __attribute__((always_inline)) inline

#define WHILE_TIMEOUT(condition, timeout) \
  for (uint32_t start = time_millis(); (condition) && (time_millis() - start) < (timeout);)

#define LOG2_8BIT(v) (8 - 90 / (((v) / 4 + 14) | 1) - 2 / ((v) / 2 + 1))
#define LOG2_16BIT(v) (8 * ((v) > 255) + LOG2_8BIT((v) >> 8 * ((v) > 255)))
#define LOG2_32BIT(v) (16 * ((v) > 65535L) + LOG2_16BIT((v) * 1L >> 16 * ((v) > 65535L)))

#define SWAP(x, y)          \
  {                         \
    __typeof__(x) temp = x; \
    x = y;                  \
    y = temp;               \
  }

template <typename T, typename L, typename U>
constexpr auto constrain(T val, L lower, U upper) {
  // Unary + preserves the C conditional operator's integer promotions.
  using lower_type = std::common_type_t<decltype(+val), decltype(+lower)>;
  using upper_type = std::common_type_t<decltype(+val), decltype(+upper)>;
  return (lower_type(val) < lower_type(lower)) ? +lower : ((upper_type(val) > upper_type(upper)) ? +upper : +val);
}

template <typename T, typename U>
constexpr auto MIN(T a, U b) {
  using common_type = std::common_type_t<decltype(+a), decltype(+b)>;
  return common_type(a) < common_type(b) ? +a : +b;
}

template <typename T, typename U>
constexpr auto MAX(T a, U b) {
  using common_type = std::common_type_t<decltype(+a), decltype(+b)>;
  return common_type(a) > common_type(b) ? +a : +b;
}

#define MHZ_TO_HZ(mhz) (mhz * 1000000)

#define MAKE_SEMVER(major, minor, patch) ((major << 16) | (minor << 8) | patch)

// workaround for vscode not reconginzing this synatx
#define RANGE_INIT(start, end) start...(end - 1)

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

float atan2approx(float y, float x);
int ipow(int base, int exp);
float fastsin(float x);
float fastcos(float x);

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2);
int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2);

uint32_t get_chip_uid();
