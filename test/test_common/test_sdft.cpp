#include <unity.h>

#include "util/sdft.h"

void test_sdft_explicit_period_update() {
  sdft_t sdft;
  sdft_init(&sdft, 0);
  TEST_ASSERT_FALSE(sdft_push(&sdft, 1.0f));
  TEST_ASSERT_FALSE(sdft_update(&sdft));
  sdft_update_period(&sdft, 125.0f);
  sdft_push(&sdft, 1.0f);
  sdft.notch_hz[0] = 250.0f;
  const sdft_t previous = sdft;
  sdft_update_period(&sdft, 125.0f);
  TEST_ASSERT_EQUAL_MEMORY(&previous, &sdft, sizeof(sdft));
  sdft_update_period(&sdft, 250.0f);
  TEST_ASSERT_EQUAL_FLOAT(250.0f, sdft.sample_period_us);
  TEST_ASSERT_EQUAL_FLOAT(250.0f, sdft.notch_hz[0]);
  TEST_ASSERT_EQUAL_UINT32(0, sdft.sample_count);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, sdft.sample_accumulator);
  for (const auto sample : sdft.samples) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sample);
  }
  for (unsigned i = 0; i < 2; i++) {
    TEST_ASSERT_FALSE(sdft_push(&sdft, 1.0f));
  }
  TEST_ASSERT_TRUE(sdft_push(&sdft, 1.0f));
}
