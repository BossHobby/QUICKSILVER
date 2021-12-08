#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX12XX)

#define NR_FHSS_ENTRIES (sizeof(fhss_freqs) / sizeof(uint32_t))
#define NR_SEQUENCE_ENTRIES 256

#define FREQ_STEP 61.03515625
#define FHSS_RNG_MAX 0x7FFF

#define FREQ_CORRECTION_MAX ((int32_t)(100000 / FREQ_STEP))
#define FREQ_CORRECTION_MIN ((int32_t)(-100000 / FREQ_STEP))

#define FREQ_HZ_TO_REG_VAL(freq) ((uint32_t)((double)freq / (double)FREQ_STEP))

const uint32_t fhss_freqs[] = {
    FREQ_HZ_TO_REG_VAL(863275000), // band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(863800000),
    FREQ_HZ_TO_REG_VAL(864325000),
    FREQ_HZ_TO_REG_VAL(864850000),
    FREQ_HZ_TO_REG_VAL(865375000), // Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(865900000),
    FREQ_HZ_TO_REG_VAL(866425000),
    FREQ_HZ_TO_REG_VAL(866950000),
    FREQ_HZ_TO_REG_VAL(867475000),
    FREQ_HZ_TO_REG_VAL(868000000),
    FREQ_HZ_TO_REG_VAL(868525000), // Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
    FREQ_HZ_TO_REG_VAL(869050000),
    FREQ_HZ_TO_REG_VAL(869575000),
};

uint8_t fhss_index = 0;

static int32_t freq_correction = 0;

static uint8_t fhss_sequence[NR_SEQUENCE_ENTRIES] = {0};
static uint32_t fhss_rng_seed = 0;

static int32_t fhss_rng() {
  uint32_t m = 2147483648;
  int32_t a = 214013;
  int32_t c = 2531011;
  fhss_rng_seed = (a * fhss_rng_seed + c) % m;
  return fhss_rng_seed >> 16;
}

static uint32_t fhss_rng_max(uint32_t max) {
  uint32_t x = fhss_rng();
  return (x * max) / FHSS_RNG_MAX;
}

static void fhss_reset_is_available(uint8_t *is_available) {
  // channel 0 is the sync channel and is never considered available
  is_available[0] = 0;

  for (uint32_t i = 1; i < NR_FHSS_ENTRIES; i++)
    is_available[i] = 1;
}

void fhss_randomize(int32_t seed) {
  uint8_t is_available[NR_FHSS_ENTRIES];
  fhss_reset_is_available(is_available);

  // Fill the FHSSsequence with channel indices
  // The 0 index is special - the 'sync' channel. The sync channel appears every
  // syncInterval hops. The other channels are randomly distributed between the
  // sync channels
  const int32_t SYNC_INTERVAL = NR_FHSS_ENTRIES - 1;

  // for each slot in the sequence table
  for (int32_t i = 0, prev = 0, left = NR_FHSS_ENTRIES - 1; i < NR_SEQUENCE_ENTRIES; i++) {
    if (i % SYNC_INTERVAL == 0) {
      // assign sync channel 0
      fhss_sequence[i] = 0;
      prev = 0;
      continue;
    }

    // pick one of the available channels. May need to loop to avoid repeats
    uint32_t index = 0;
    do {
      int32_t c = fhss_rng_max(left); // returnc 0 < c <left
      // find the c'th entry in the isAvailable array
      // can skip 0 as that's the sync channel and is never available for normal allocation
      index = 1;

      int32_t found = 0;
      while (index < NR_FHSS_ENTRIES) {
        if (is_available[index]) {
          if (found == c)
            break;
          found++;
        }
        index++;
      }
      if (index == NR_FHSS_ENTRIES) {
        index = 0;
        break;
      }
    } while (index == prev); // can't use index if it repeats the previous value

    fhss_sequence[i] = index; // assign the value to the sequence array
    is_available[index] = 0;  // clear the flag
    prev = index;             // remember for next iteration
    left--;                   // reduce the count of available channels
    if (left == 0) {
      // we've assigned all of the channels, so reset for next cycle
      fhss_reset_is_available(is_available);
      left = NR_FHSS_ENTRIES - 1;
    }
  }
}

uint32_t fhss_get_freq(uint16_t index) {
  return fhss_freqs[index] - freq_correction;
}

uint32_t fhss_next_freq() {
  return fhss_get_freq(fhss_sequence[fhss_index++]);
}

int32_t fhss_update_freq_correction(uint8_t value) {
  if (!value) {
    if (freq_correction < FREQ_CORRECTION_MAX) {
      freq_correction += 1; //min freq step is ~ 61hz but don't forget we use FREQ_HZ_TO_REG_VAL so the units here are not hz!
    } else {
      freq_correction = 0; //reset because something went wrong
    }
  } else {
    if (freq_correction > FREQ_CORRECTION_MIN) {
      freq_correction -= 1; //min freq step is ~ 61hz
    } else {
      freq_correction = 0; //reset because something went wrong
    }
  }

  const current_freq = fhss_get_freq(fhss_sequence[fhss_index]);
  return (freq_correction * 1e6 / current_freq) * 95 / 100;
}

void fhss_reset() {
  freq_correction = 0;
}

#endif