#include <math.h>
#include <string.h>
#include <unity.h>

// Include mock helpers
#include "mock_helpers.h"

// Include filter module
#include "control/control.h"
#include "core/profile.h"
#include "driver/time.h"
#include "util/filter.h"

// Test fixtures
void test_notch_matches_reference_with_changing_frequency() {
  const float periods[] = {125, 250, 500};
  for (float period : periods) {
    filter_biquad_notch_t filters[3];
    filter_biquad_state_t history[3];
    double x1[3] = {}, x2[3] = {}, y1[3] = {}, y2[3] = {};
    for (unsigned p = 0; p < 3; p++) {
      filter_biquad_notch_init(&filters[p], &history[p], 1, 100 + p * 200, period);
    }
    for (unsigned i = 0; i < 12000; i++) {
      if (i % 97 == 0) {
        for (unsigned p = 0; p < 3; p++) {
          // Include low centres, moving peaks, and disabling/re-enabling.
          const float hz = i >= 4000 && i < 4200 ? 0 : 100 + (i / 97 + p * 200) % 501;
          filter_biquad_notch_coeff(&filters[p], hz, period);
        }
      }
      float actual = 10 * sinf(i * 0.11f) + 5 * cosf(i * 0.73f) + (i % 251 == 0 ? 20 : 0);
      double expected = actual;
      for (unsigned p = 0; p < 3; p++) {
        const auto &f = filters[p];
        if (f.hz >= 0.1f) {
          // General direct-form equation, evaluated independently in double.
          const double result = f.b0 * expected + f.b1 * x1[p] + f.b2 * x2[p] - f.a1 * y1[p] - f.a2 * y2[p];
          x2[p] = x1[p];
          x1[p] = expected;
          y2[p] = y1[p];
          y1[p] = result;
          expected = result;
        }
        actual = filter_biquad_notch_step(&filters[p], &history[p], actual);
      }
      TEST_ASSERT_FLOAT_WITHIN(0.002f, expected, actual);
    }
  }
}

static void filter_setUp(void) {
  // Reset hardware mocks before each test
  mock_hardware_reset_all();

  // Initialize profile and state
  memset(&profile, 0, sizeof(profile));
  memset(&state, 0, sizeof(state));

  // Set default looptime
  state.looptime = 0.000125f; // 8kHz
}


// Test filter initialization
void test_filter_init(void) {
  filter_setUp();
  // Initialize filter with 100Hz cutoff at 8kHz
  filter_t filter;
  filter_state_t state;
  filter_init(FILTER_LP_PT1, &filter, &state, 1, 100.0f, 125.0f); // 125 microseconds

  // Check that filter state is initialized
  TEST_ASSERT_NOT_NULL(&filter);
}

// Test low-pass filtering
void test_filter_lowpass_pt1(void) {
  filter_setUp();
  filter_t filter;
  filter_state_t filter_state;
  filter_init(FILTER_LP_PT1, &filter, &filter_state, 1, 100.0f, 125.0f);

  // Apply step input
  float input = 1.0f;
  float output = 0.0f;

  // Run filter several times to see convergence
  for (int i = 0; i < 100; i++) {
    output = filter_step(FILTER_LP_PT1, &filter, &filter_state, input);
  }

  // Output should converge toward input
  TEST_ASSERT_FLOAT_WITHIN(0.1f, input, output);
}

// Test high-frequency attenuation
void test_filter_highfreq_attenuation(void) {
  filter_setUp();
  filter_t filter;
  filter_state_t filter_state;
  filter_init(FILTER_LP_PT1, &filter, &filter_state, 1, 100.0f, 125.0f);

  // Apply high-frequency signal
  float output_mag = 0.0f;
  for (int i = 0; i < 1000; i++) {
    float input = sinf(2.0f * M_PI * 1000.0f * i * 0.000125f); // 1kHz signal
    float output = filter_step(FILTER_LP_PT1, &filter, &filter_state, input);
    output_mag += output * output;
  }

  // High frequency should be attenuated
  TEST_ASSERT_LESS_THAN_FLOAT(0.5f, output_mag / 1000.0f);
}

// Test filter reset
void test_filter_reset(void) {
  filter_setUp();
  filter_t filter;
  filter_state_t filter_state;
  filter_init(FILTER_LP_PT1, &filter, &filter_state, 1, 100.0f, 125.0f);

  // Process some data
  for (int i = 0; i < 10; i++) {
    filter_step(FILTER_LP_PT1, &filter, &filter_state, 1.0f);
  }

  // Reset state and process again
  filter_init_state(&filter_state, 1);
  float output = filter_step(FILTER_LP_PT1, &filter, &filter_state, 0.0f);

  // After reset, output should be close to new input
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output);
}

// Test different filter types
void test_filter_types(void) {
  filter_setUp();
  filter_t filter_pt1, filter_pt2;
  filter_state_t state_pt1, state_pt2;

  filter_init(FILTER_LP_PT1, &filter_pt1, &state_pt1, 1, 100.0f, 125.0f);
  filter_init(FILTER_LP_PT2, &filter_pt2, &state_pt2, 1, 100.0f, 125.0f);

  // Apply same input to both
  float input = 1.0f;
  float output_pt1 = filter_step(FILTER_LP_PT1, &filter_pt1, &state_pt1, input);
  float output_pt2 = filter_step(FILTER_LP_PT2, &filter_pt2, &state_pt2, input);

  // PT2 should have different response than PT1
  TEST_ASSERT_NOT_EQUAL(output_pt1, output_pt2);
}

// Test filter cascade
void test_filter_cascade(void) {
  filter_setUp();
  filter_t filter1, filter2;
  filter_state_t state1, state2;

  filter_init(FILTER_LP_PT1, &filter1, &state1, 1, 200.0f, 125.0f);
  filter_init(FILTER_LP_PT1, &filter2, &state2, 1, 100.0f, 125.0f);

  // Apply cascaded filtering
  float input = 1.0f;
  float intermediate = filter_step(FILTER_LP_PT1, &filter1, &state1, input);
  float output = filter_step(FILTER_LP_PT1, &filter2, &state2, intermediate);

  // Cascaded output should be more filtered
  TEST_ASSERT_LESS_THAN_FLOAT(intermediate, output);
}

