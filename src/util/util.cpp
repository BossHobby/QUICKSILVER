#include "util/util.h"

#include <math.h>
#include <string.h>

#include "core/project.h"
#include "driver/time.h"

#define sinPolyCoef3 -1.666665710e-1f // Double: -1.666665709650470145824129400050267289858e-1
#define sinPolyCoef5 8.333017292e-3f  // Double:  8.333017291562218127986291618761571373087e-3
#define sinPolyCoef7 -1.980661520e-4f // Double: -1.980661520135080504411629636078917643846e-4
#define sinPolyCoef9 2.600054768e-6f  // Double:  2.600054767890361277123254766503271638682e-6

// Rational atan approximation used by Betaflight's common/maths.c.
static constexpr float ATAN_COEFFICIENTS[] = {3.14551665884836e-07f, 0.99997356613987f, 0.14744007058297684f, 0.3099814292351353f, 0.05030176425872175f, 0.1471039133652469f, 0.6444640676891548f};

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}

float fastsin(float x) {
  const int32_t xint = x;

  if (xint < -32 || xint > 32)
    return 0.0f; // Stop here on error input (5 * 360 Deg)

  while (x > M_PI_F)
    x -= (2.0f * M_PI_F); // always wrap input angle to -PI..PI

  while (x < -M_PI_F)
    x += (2.0f * M_PI_F);

  if (x > (0.5f * M_PI_F))
    x = (0.5f * M_PI_F) - (x - (0.5f * M_PI_F)); // We just pick -90..+90 Degree

  else if (x < -(0.5f * M_PI_F))
    x = -(0.5f * M_PI_F) - ((0.5f * M_PI_F) + x);

  const float x2 = x * x;
  return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float fastcos(float x) {
  return fastsin(x + (0.5f * M_PI_F));
}

float normalize_deg(float deg) {
  while (deg > 360.f)
    deg -= 360.f;
  while (deg < 0.f)
    deg += 360.f;
  return deg;
}

float normalize_rad(float rad) {
  while (rad > M_PI_F * 2.f)
    rad -= M_PI_F * 2.f;
  while (rad < 0.f)
    rad += M_PI_F * 2.f;
  return rad;
}

int ipow(int base, int exp) {
  int result = 1;
  for (;;) {
    if (exp & 1)
      result *= base;
    exp >>= 1;
    if (!exp)
      break;
    base *= base;
  }

  return result;
}

float atan2approx_rad(float y, float x) {
  if (y == 0.0f) return x < 0.0f ? copysignf(M_PI_F, y) : y;
  if (x == 0.0f) return copysignf(M_PI_F * 0.5f, y);

  const float abs_x = fabsf(x);
  const float abs_y = fabsf(y);
  const float largest = MAX(abs_x, abs_y);
  const float ratio = MIN(abs_x, abs_y) / largest;
  const float numerator = -((((ATAN_COEFFICIENTS[4] * ratio - ATAN_COEFFICIENTS[3]) * ratio - ATAN_COEFFICIENTS[2]) * ratio - ATAN_COEFFICIENTS[1]) * ratio - ATAN_COEFFICIENTS[0]);
  const float denominator = (ATAN_COEFFICIENTS[6] * ratio + ATAN_COEFFICIENTS[5]) * ratio + 1.0f;
  float angle = numerator / denominator;
  if (abs_y > abs_x) angle = M_PI_F * 0.5f - angle;
  if (x < 0.0f) angle = M_PI_F - angle;
  if (y < 0.0f) angle = -angle;
  return angle;
}

float atan2approx(float y, float x) {
  return atan2approx_rad(y, x) * RADTODEG;
}

int8_t buf_equal(const uint8_t *str1, size_t len1, const uint8_t *str2, size_t len2) {
  if (len2 != len1) {
    return 0;
  }
  for (size_t i = 0; i < len1; i++) {
    if (str1[i] != str2[i]) {
      return 0;
    }
  }
  return 1;
}

int8_t buf_equal_string(const uint8_t *str1, size_t len1, const char *str2) {
  return buf_equal(str1, len1, (const uint8_t *)str2, strlen(str2));
}

uint32_t get_chip_uid() {
#ifdef SIMULATOR
  return 0xdeedbeef;
#else
  return ((uint32_t *)UID_BASE)[0] ^ ((uint32_t *)UID_BASE)[1] ^ ((uint32_t *)UID_BASE)[1];
#endif
}
