#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#define ELRS_CRC_POLY 0x07     // 0x83
#define ELRS_CRC14_POLY 0x2E57 // 0x372B

#define ELRS_LQ_SIZE ((100 + 31) / 32)

#define CRC_LENGTH 256

typedef struct {
  uint8_t val;
  uint8_t byte;

  uint32_t mask;
  uint32_t buffer[ELRS_LQ_SIZE];
} elrs_lq_t;

static elrs_lq_t lq;

static uint16_t crc14tab[CRC_LENGTH];

void crc14_init() {
  for (uint16_t i = 0; i < CRC_LENGTH; i++) {
    uint16_t crc = i << (14 - 8);
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc << 1) ^ ((crc & 0x2000) ? ELRS_CRC14_POLY : 0);
    }
    crc14tab[i] = crc;
  }
}

uint16_t crc14_calc(const volatile uint8_t *data, uint8_t len, uint16_t crc) {
  while (len--) {
    crc = (crc << 8) ^ crc14tab[((crc >> 6) ^ (uint16_t)*data++) & 0x00FF];
  }
  return crc & 0x3FFF;
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
  if ((lq.byte == 3) && (lq.mask & (1 << ELRS_LQ_SIZE))) {
    lq.byte = 0;
    lq.mask = (1 << 0);
  }

  if ((lq.buffer[lq.byte] & lq.mask) != 0) {
    lq.buffer[lq.byte] &= ~lq.mask;
    lq.val -= 1;
  }
}

uint8_t elrs_lq_get() {
  return lq.val;
}

bool elrs_lq_current_is_set() {
  return lq.buffer[lq.byte] & lq.mask;
}

void elrs_lq_reset() {
  lq.val = 0;
  lq.byte = 0;
  lq.mask = (1 << 0);

  for (uint32_t i = 0; i < ELRS_LQ_SIZE; i++) {
    lq.buffer[i] = 0;
  }
}

uint8_t tlm_ratio_enum_to_value(expresslrs_tlm_ratio_t val) {
  switch (val) {
  case TLM_RATIO_NO_TLM:
    return 1;
    break;
  case TLM_RATIO_1_2:
    return 2;
    break;
  case TLM_RATIO_1_4:
    return 4;
    break;
  case TLM_RATIO_1_8:
    return 8;
    break;
  case TLM_RATIO_1_16:
    return 16;
    break;
  case TLM_RATIO_1_32:
    return 32;
    break;
  case TLM_RATIO_1_64:
    return 64;
    break;
  case TLM_RATIO_1_128:
    return 128;
    break;
  default:
    return 0;
  }
}

uint16_t rate_enum_to_hz(expresslrs_rf_rates_t val) {
  switch (val) {
  case RATE_500HZ:
    return 500;
  case RATE_250HZ:
    return 250;
  case RATE_200HZ:
    return 200;
  case RATE_150HZ:
    return 150;
  case RATE_100HZ:
    return 100;
  case RATE_50HZ:
    return 50;
  case RATE_25HZ:
    return 25;
  case RATE_4HZ:
    return 4;
  default:
    return 1;
  }
}

#endif