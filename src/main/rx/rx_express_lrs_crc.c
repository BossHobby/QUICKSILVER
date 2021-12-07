#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX12XX)

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

#endif