#include <math.h>
#include <stdio.h>
#include <unity.h>

#include "util/util.h"

void test_atan2approx_accuracy() {
  constexpr double pi = 3.14159265358979323846L;
  const float scales[] = {1e-20f, 1e-6f, 1.0f, 1e6f, 1e20f};
  double max_error = 0.0;
  for (float scale : scales) {
    for (int step = -36000; step <= 36000; step++) {
      const double angle = double(step) * pi / double(36000);
      const float x = float(double(scale) * cos(angle));
      const float y = float(double(scale) * sin(angle));
      const double error = fabs(remainder(double(atan2approx_rad(y, x)) - atan2(double(y), double(x)), double(2) * pi));
      if (error > max_error) max_error = error;
    }
  }
  printf("atan2 approximation max error: %.9f degrees\n", max_error * double(180) / pi);
  TEST_ASSERT_TRUE(max_error < double(0.000002L)); // Less than 0.00012 degrees.
  TEST_ASSERT_EQUAL_FLOAT(0.0f, atan2approx_rad(0.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.000002f, 0.0f, atan2approx_rad(0.0f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.000002f, M_PI_F, atan2approx_rad(0.0f, -1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.000002f, M_PI_F * 0.5f, atan2approx_rad(1.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.000002f, -M_PI_F * 0.5f, atan2approx_rad(-1.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.00012f, 90.0f, atan2approx(1.0f, 0.0f));
}
