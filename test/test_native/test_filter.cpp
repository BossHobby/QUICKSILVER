#include <math.h>
#include <string.h>
#include <unity.h>

// Include mock helpers
#include "mock_helpers.h"

// Include filter module
#include "core/profile.h"
#include "driver/time.h"
#include "flight/control.h"
#include "flight/filter.h"

// Test fixtures
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

