#include <math.h>
#include <string.h>
#include <unity.h>

#include "util/sdft.h"
#include "util/util.h"

static constexpr float periods[] = {125.0f, 250.0f, 312.5f, 500.0f};
static constexpr float frequencies[] = {100.0f, 117.0f, 300.0f, 550.0f, 600.0f};

void test_sdft_batches_match_direct_transform() {
  for (float period : periods) {
    sdft_t sdft;
    sdft_init(&sdft, period);
    float samples[SDFT_SAMPLE_SIZE] = {};
    uint32_t idx = 0;
    float sum = 0;
    uint32_t blocks = 0;
    for (uint32_t n = 0; n < 120 * sdft.sub_samples; n++) {
      const float t = n * period * 1e-6f;
      const float value = sinf(2.0f * M_PI_F * 117.0f * t) + 0.7f * cosf(2.0f * M_PI_F * 557.0f * t);
      sum += value;
      if (!sdft_push(&sdft, value)) {
        continue;
      }
      // The completed spectrum contains the previous block; the newest
      // block is latched for processing during the next group of pushes.
      if (++blocks % 10 == 0) {
        for (uint32_t k = sdft.bin_min_index - 2; k <= sdft.bin_max_index + 2; k++) {
          float re = 0, im = 0;
          for (uint32_t age = 0; age < SDFT_SAMPLE_SIZE; age++) {
            const float value = samples[(idx + SDFT_SAMPLE_SIZE - 1 - age) % SDFT_SAMPLE_SIZE];
            const float angle = 2.0f * M_PI_F * k * (age + 1) / SDFT_SAMPLE_SIZE;
            const float weighted = value * powf(SDFT_DAMPING_FACTOR, age);
            re += weighted * cosf(angle);
            im += weighted * sinf(angle);
          }
          TEST_ASSERT_FLOAT_WITHIN(0.003f, re, __real__ sdft.data[k]);
          TEST_ASSERT_FLOAT_WITHIN(0.003f, im, __imag__ sdft.data[k]);
        }
      }
      samples[idx] = sum / sdft.sub_samples;
      idx = (idx + 1) % SDFT_SAMPLE_SIZE;
      sum = 0;
    }
  }
}

static void push_axes(sdft_t axes[SDFT_AXES], uint8_t *current_axis, float phase) {
  for (uint32_t axis = 0; axis < SDFT_AXES; axis++) {
    if (sdft_push(&axes[axis], sinf(phase + axis * 0.7f)) && *current_axis == SDFT_AXES) {
      *current_axis = 0;
    }
  }
  if (*current_axis < SDFT_AXES && sdft_update(&axes[*current_axis])) {
    (*current_axis)++;
  }
}

static void assert_tone(const sdft_t *sdft, float frequency, float tolerance) {
  float error = SDFT_MAX_HZ;
  for (uint32_t peak = 0; peak < SDFT_PEAKS; peak++) {
    TEST_ASSERT_TRUE(isfinite(sdft->notch_hz[peak]));
    error = MIN(error, fabsf(sdft->notch_hz[peak] - frequency));
  }
  TEST_ASSERT_LESS_OR_EQUAL_FLOAT(tolerance, error);
}

void test_sdft_tracks_tones_across_sample_rates() {
  for (float period : periods) {
    for (float frequency : frequencies) {
      sdft_t axes[SDFT_AXES];
      for (auto &axis : axes) {
        sdft_init(&axis, period);
      }
      uint8_t current_axis = SDFT_AXES;
      for (uint32_t n = 0; n < (uint32_t)(1e6f / period); n++) {
        push_axes(axes, &current_axis, 2.0f * M_PI_F * frequency * n * period * 1e-6f);
      }
      for (const auto &axis : axes) {
        assert_tone(&axis, frequency, 5.0f);
      }
    }
  }
}

void test_sdft_tracks_frequency_sweep() {
  for (float period : periods) {
    sdft_t axes[SDFT_AXES];
    for (auto &axis : axes) {
      sdft_init(&axis, period);
    }
    uint8_t current_axis = SDFT_AXES;
    float phase = 0;
    for (uint32_t n = 0; n < (uint32_t)(2e6f / period); n++) {
      const float t = n * period * 1e-6f;
      const float frequency = 150.0f + 200.0f * t;
      phase = fmodf(phase + 2.0f * M_PI_F * frequency * period * 1e-6f, 2.0f * M_PI_F);
      push_axes(axes, &current_axis, phase);
      if (t > 0.5f && n % 100 == 0) {
        for (const auto &axis : axes) {
          assert_tone(&axis, frequency, 20.0f);
        }
      }
    }
  }
}

void test_sdft_waits_for_complete_spectrum() {
  sdft_t sdft;
  sdft_init(&sdft, 125.0f);
  sdft_push(&sdft, 1.0f);
  TEST_ASSERT_FALSE(sdft_update(&sdft));
  TEST_ASSERT_EQUAL(SDFT_UPDATE_MAGNITUDE, sdft.state);
  while (sdft.sample_count != 0) {
    sdft_push(&sdft, 1.0f);
  }
  sdft_update(&sdft);
  TEST_ASSERT_EQUAL(SDFT_DETECT_PEAKS, sdft.state);
}

void test_sdft_resets_history_on_sample_rate_change() {
  sdft_t axes[SDFT_AXES];
  for (auto &axis : axes) {
    memset(&axis, 0xff, sizeof(axis));
    sdft_init(&axis, 0);
    TEST_ASSERT_FALSE(sdft_push(&axis, 1.0f));
    TEST_ASSERT_FALSE(sdft_update(&axis));
  }
  uint8_t current_axis = SDFT_AXES;
  for (float period : periods) {
    const float previous_notch = axes[0].notch_hz[0];
    for (auto &axis : axes) {
      sdft_update_period(&axis, period);
    }
    push_axes(axes, &current_axis, 0.5f);
    TEST_ASSERT_EQUAL_FLOAT(previous_notch, axes[0].notch_hz[0]);
    for (uint32_t k = 0; k < SDFT_BIN_COUNT; k++) {
      TEST_ASSERT_EQUAL_FLOAT(0, __real__ axes[0].data[k]);
      TEST_ASSERT_EQUAL_FLOAT(0, __imag__ axes[0].data[k]);
    }
    for (uint32_t n = 0; n < (uint32_t)(1e6f / period); n++) {
      push_axes(axes, &current_axis, 2.0f * M_PI_F * 550.0f * n * period * 1e-6f);
    }
    for (const auto &axis : axes) {
      assert_tone(&axis, 550.0f, 5.0f);
    }
    const sdft_t previous = axes[0];
    sdft_update_period(&axes[0], period);
    TEST_ASSERT_EQUAL_MEMORY(&previous, &axes[0], sizeof(previous));
  }
}
