#include "rx/express_lrs.h"

#include "core/project.h"

#if defined(RX_EXPRESS_LRS)

#define ELRS_CRC_POLY 0x07 // 0x83

#define ELRS_LQ_N 100
#define ELRS_LQ_SIZE ((ELRS_LQ_N + 31) / 32)

#define CRC_LENGTH 256

typedef struct {
  uint8_t val;
  uint8_t byte;
  uint8_t count;

  uint32_t mask;
  uint32_t buffer[ELRS_LQ_SIZE];
} elrs_lq_t;

static elrs_lq_t lq;

typedef struct {
  int32_t acc;
  int32_t count;
} elrs_snr_mean_t;

static elrs_snr_mean_t snr_mean;

static uint16_t crc_tab[CRC_LENGTH];
static uint8_t crc_bits = 0;
static uint16_t crc_poly = 0;
static uint16_t crc_bitmask = 0;

void elrs_crc_init(uint8_t bits, uint16_t poly) {
  if (crc_poly == poly && crc_bits == bits) {
    return;
  }

  crc_poly = poly;
  crc_bits = bits;
  crc_bitmask = (1 << crc_bits) - 1;

  uint16_t crc = 0;
  const uint16_t highbit = 1 << (crc_bits - 1);

  for (uint16_t i = 0; i < CRC_LENGTH; i++) {
    crc = i << (bits - 8);
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc << 1) ^ ((crc & highbit) ? poly : 0);
    }
    crc_tab[i] = crc;
  }
}

uint16_t elrs_crc_calc(const volatile uint8_t *data, uint8_t len, uint16_t crc) {
  while (len--) {
    crc = (crc << 8) ^ crc_tab[((crc >> (crc_bits - 8)) ^ (uint16_t)*data++) & 0x00FF];
  }
  return crc & crc_bitmask;
}

void elrs_lpf_init(elrs_lpf_t *lpf, int32_t beta) {
  lpf->beta = beta;
  lpf->fp_shift = 5;
  lpf->smooth_data_int = 0;
  lpf->smooth_data_fp = 0;
}

int32_t elrs_lpf_update(elrs_lpf_t *lpf, int32_t data) {
  int32_t raw_data = data;
  raw_data <<= lpf->fp_shift; // Shift to fixed point

  lpf->smooth_data_fp = (lpf->smooth_data_fp << lpf->beta) - lpf->smooth_data_fp;
  lpf->smooth_data_fp += raw_data;
  lpf->smooth_data_fp >>= lpf->beta;

  // Don't do the following shift if you want to do further
  // calculations in fixed-point using SmoothData
  lpf->smooth_data_int = lpf->smooth_data_fp >> lpf->fp_shift;

  return lpf->smooth_data_int;
}

void elrs_lq_add() {
  if (elrs_lq_current_is_set()) {
    return;
  }
  lq.buffer[lq.byte] |= lq.mask;
  lq.val += 1;
}

void elrs_lq_inc() {
  // Increment the counter by shifting one bit higher
  // If we've shifted out all the bits, move to next idx
  lq.mask <<= 1;
  if (lq.mask == 0) {
    lq.mask = (1 << 0);
    lq.byte += 1;
  }

  // At idx N / 32 and bit N % 32, wrap back to idx=0, bit=0
  if ((lq.byte == (ELRS_LQ_N / 32)) && (lq.mask & (1 << (ELRS_LQ_N % 32)))) {
    lq.byte = 0;
    lq.mask = (1 << 0);
  }

  if ((lq.buffer[lq.byte] & lq.mask) != 0) {
    lq.buffer[lq.byte] &= ~lq.mask;
    lq.val -= 1;
  }

  if (lq.count < ELRS_LQ_N) {
    ++lq.count;
  }
}

uint8_t elrs_lq_get() {
  return (uint32_t)lq.val * 100U / lq.count;
}

uint8_t elrs_lq_get_raw() {
  return lq.val;
}

bool elrs_lq_current_is_set() {
  return lq.buffer[lq.byte] & lq.mask;
}

void elrs_lq_reset() {
  lq.val = 0;
  lq.byte = 0;
  lq.count = 1;
  lq.mask = (1 << 0);

  for (uint32_t i = 0; i < ELRS_LQ_SIZE; i++) {
    lq.buffer[i] = 0;
  }
}

void elrs_snr_mean_reset() {
  snr_mean.acc = 0;
  snr_mean.count = 0;
}

void elrs_snr_mean_add(const int8_t val) {
  snr_mean.acc += val;
  snr_mean.count++;
}

int8_t elrs_snr_mean_get(const int8_t def) {
  if (snr_mean.count) {
    int8_t ret = snr_mean.acc / snr_mean.count;
    elrs_snr_mean_reset();
    return ret;
  }
  return def;
}

uint8_t tlm_ratio_enum_to_value(expresslrs_tlm_ratio_t val) {
  // !! TLM_RATIO_STD/TLM_RATIO_DISARMED should be converted by the caller !!
  if (val == TLM_RATIO_NO_TLM)
    return 1;

  // 1 << (8 - (val - TLM_RATIO_NO_TLM))
  // 1_128 = 128, 1_64 = 64, 1_32 = 32, etc
  return 1 << (8 + TLM_RATIO_NO_TLM - val);
}

uint16_t rate_enum_to_hz(expresslrs_rf_rates_t val) {
  switch (val) {
  case RATE_FLRC_1000HZ:
    return 1000;
  case RATE_FLRC_500HZ:
    return 500;
  case RATE_DVDA_500HZ:
    return 500;
  case RATE_DVDA_250HZ:
    return 250;
  case RATE_LORA_500HZ:
    return 500;
  case RATE_LORA_333HZ_8CH:
    return 333;
  case RATE_LORA_250HZ:
    return 250;
  case RATE_LORA_200HZ:
    return 200;
  case RATE_LORA_150HZ:
    return 150;
  case RATE_LORA_100HZ:
    return 100;
  case RATE_LORA_100HZ_8CH:
    return 100;
  case RATE_LORA_50HZ:
    return 50;
  case RATE_LORA_25HZ:
    return 25;
  case RATE_LORA_4HZ:
    return 4;
  default:
    return 1;
  }
}

#endif