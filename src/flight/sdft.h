#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <complex.h>
#undef I

#define SDFT_AXES 3
#define SDFT_PEAKS 3

#define SDFT_FILTER_HZ 4

#define SDFT_MIN_HZ 80
#define SDFT_MAX_HZ 500

#define SDFT_SAMPLE_SIZE 100
#define SDFT_BIN_COUNT (SDFT_SAMPLE_SIZE / 2)

#define SDFT_DAMPING_FACTOR 0.9999f
#define SDFT_SAMPLE_HZ (1e6f / LOOPTIME)

#define SDFT_SUBSAMPLES (SDFT_SAMPLE_HZ / (2 * SDFT_MAX_HZ))
#define SDFT_HZ_RESOLUTION ((SDFT_SAMPLE_HZ / SDFT_SUBSAMPLES) / SDFT_SAMPLE_SIZE)

typedef float complex complex_float;

typedef enum {
  SDFT_UPDATE_MAGNITUE,
  SDFT_DETECT_PEAKS,
  SDFT_CALC_FREQ,
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

  float noise_floor;
  float magnitude[SDFT_BIN_COUNT];

  float peak_values[SDFT_PEAKS];
  uint32_t peak_indicies[SDFT_PEAKS];

  float notch_hz[SDFT_PEAKS];
} sdft_t;

void sdft_init(sdft_t *sdft);
bool sdft_push(sdft_t *sdft, float val);
bool sdft_update(sdft_t *sdft);