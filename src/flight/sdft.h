#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <complex.h>
#undef I

#define SDFT_AXES 3
#define SDFT_PEAKS 3

#define SDFT_MIN_HZ 80
#define SDFT_MAX_HZ 500 // limit: 500 Hz if SAMPLE_PERIOD is 1 ms.

#define SDFT_SAMPLE_PERIOD 1000                        // us. Sampling every 1 ms is enough for MAX_HZ up to 500.
#define SDFT_SAMPLE_SIZE (100000 / SDFT_SAMPLE_PERIOD) // 0.1 seconds time window gives 10 Hz resolution.
#define SDFT_BIN_COUNT (SDFT_SAMPLE_SIZE / 2)

typedef float complex complex_float;

typedef enum {
  SDFT_WAIT_FOR_SAMPLES,
  SDFT_UPDATE_MAGNITUE,
  SDFT_DETECT_PEAKS,
  SDFT_UPDATE_FILTERS,
} sdft_state_t;

typedef struct {
  sdft_state_t state;

  uint32_t idx;

  float sample_accumulator;
  float sample_avg;
  uint32_t sample_count;

  float samples[SDFT_SAMPLE_SIZE];
  complex_float data[SDFT_BIN_COUNT];

  float magnitude[SDFT_BIN_COUNT];
  float notch_hz[SDFT_PEAKS];
} sdft_t;

void sdft_init(sdft_t *sdft);
bool sdft_update(sdft_t *sdft, float val);