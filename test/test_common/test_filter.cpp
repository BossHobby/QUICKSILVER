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
    double x1[3] = {}, x2[3] = {}, y1[3] = {}, y2[3] = {};
    for (unsigned p = 0; p < 3; p++) {
      filters[p].init(100 + p * 200, period);
    }
    for (unsigned i = 0; i < 12000; i++) {
      if (i % 97 == 0) {
        for (unsigned p = 0; p < 3; p++) {
          // Include low centres, moving peaks, and disabling/re-enabling.
          const float hz = i >= 4000 && i < 4200 ? 0 : 100 + (i / 97 + p * 200) % 501;
          filters[p].coeff(hz, period);
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
        actual = filters[p].step(actual);
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
  // 100Hz cutoff at 8kHz; a fresh filter starts from zero.
  filter_t<> filter;
  filter.init(FILTER_LP_PT1, 100.0f, 125.0f);
  TEST_ASSERT_GREATER_THAN_FLOAT(0.0f, filter.step(1.0f));
}

void test_filter_coeff_applies_type_change(void) {
  // A type edit at an unchanged cutoff must not keep the old order's response.
  filter_t<> changed;
  changed.init(FILTER_LP_PT1, 100.0f, 125.0f);
  changed.coeff(FILTER_LP_PT3, 100.0f, 125.0f);

  filter_t<> fresh;
  fresh.init(FILTER_LP_PT3, 100.0f, 125.0f);
  for (int i = 0; i < 10; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, fresh.step(1.0f), changed.step(1.0f));
  }
}

// Reinitializing with another type starts from rest, like a fresh filter.
void test_filter_reconfigure_type(void) {
  for (int from = FILTER_NONE; from < FILTER_MAX; from++) {
    for (int to = FILTER_NONE; to < FILTER_MAX; to++) {
      filter_t<vec3_t> changed;
      changed.init((filter_type_t)from, 100.0f, 125.0f);
      for (int i = 0; i < 100; i++) {
        changed.step(vec3_t{{5.0f, 5.0f, 5.0f}});
      }
      changed.init((filter_type_t)to, 100.0f, 125.0f);
      filter_t<> fresh;
      fresh.init((filter_type_t)to, 100.0f, 125.0f);
      for (int i = 0; i < 10; i++) {
        const float expected = fresh.step(1.0f);
        const vec3_t actual = changed.step(vec3_t{{1.0f, 1.0f, 1.0f}});
        for (int axis = 0; axis < 3; axis++) {
          TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected, actual.axis[axis]);
        }
      }
    }
  }
}

void test_filter_lowpass_pt1(void) {
  filter_setUp();
  filter_t<> filter;
  filter.init(FILTER_LP_PT1, 100.0f, 125.0f);

  // Apply step input
  float input = 1.0f;
  float output = 0.0f;

  // Run filter several times to see convergence
  for (int i = 0; i < 100; i++) {
    output = filter.step(input);
  }

  // Output should converge toward input
  TEST_ASSERT_FLOAT_WITHIN(0.1f, input, output);
}

// Test high-frequency attenuation
void test_filter_highfreq_attenuation(void) {
  filter_setUp();
  filter_t<> filter;
  filter.init(FILTER_LP_PT1, 100.0f, 125.0f);

  // Apply high-frequency signal
  float output_mag = 0.0f;
  for (int i = 0; i < 1000; i++) {
    float input = sinf(2.0f * M_PI * 1000.0f * i * 0.000125f); // 1kHz signal
    float output = filter.step(input);
    output_mag += output * output;
  }

  // High frequency should be attenuated
  TEST_ASSERT_LESS_THAN_FLOAT(0.5f, output_mag / 1000.0f);
}

// Test filter reset
void test_filter_reset(void) {
  filter_setUp();
  filter_t<> filter;
  filter.init(FILTER_LP_PT1, 100.0f, 125.0f);

  // Process some data
  for (int i = 0; i < 10; i++) {
    filter.step(1.0f);
  }

  // Reset state and process again
  filter.reset();
  float output = filter.step(0.0f);

  // After reset, output should be close to new input
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, output);
}

// Test different filter types
void test_filter_types(void) {
  filter_setUp();
  filter_t<> filter_pt1, filter_pt2;
  filter_pt1.init(FILTER_LP_PT1, 100.0f, 125.0f);
  filter_pt2.init(FILTER_LP_PT2, 100.0f, 125.0f);

  // Apply same input to both
  float input = 1.0f;
  float output_pt1 = filter_pt1.step(input);
  float output_pt2 = filter_pt2.step(input);

  // PT2 should have different response than PT1
  TEST_ASSERT_NOT_EQUAL(output_pt1, output_pt2);
}

// Each channel of a multi-channel filter matches its own scalar filter.
void test_filter_channels_independent(void) {
  filter_t<vec4_t> channels;
  channels.init(FILTER_LP_PT3, 100.0f, 125.0f);
  filter_t<> scalar[4];
  for (auto &f : scalar) {
    f.init(FILTER_LP_PT3, 100.0f, 125.0f);
  }
  for (int i = 0; i < 50; i++) {
    const vec4_t input = {{sinf(i * 0.3f), 2.0f, -1.0f, i * 0.1f}};
    const vec4_t output = channels.step(input);
    for (int c = 0; c < 4; c++) {
      TEST_ASSERT_FLOAT_WITHIN(1e-6f, scalar[c].step(input.axis[c]), output.axis[c]);
    }
  }
}

// Test filter cascade
void test_filter_cascade(void) {
  filter_setUp();
  filter_t<> filter1, filter2;
  filter1.init(FILTER_LP_PT1, 200.0f, 125.0f);
  filter2.init(FILTER_LP_PT1, 100.0f, 125.0f);

  // Apply cascaded filtering
  float input = 1.0f;
  float intermediate = filter1.step(input);
  float output = filter2.step(intermediate);

  // Cascaded output should be more filtered
  TEST_ASSERT_LESS_THAN_FLOAT(intermediate, output);
}
