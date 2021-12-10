#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

#define ELRS_CRC_POLY 0x07     // 0x83
#define ELRS_CRC14_POLY 0x2E57 // 0x372B

#define CRC_LENGTH 256

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

uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc) {
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

#endif