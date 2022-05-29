#include "crc.h"

#define DVB_S2_POLY 0xD5

uint8_t crc8_calc(const uint8_t poly, uint8_t crc, const uint8_t input) {
  crc ^= input;

  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ DVB_S2_POLY;
    } else {
      crc = crc << 1;
    }
  }

  return crc;
}

uint8_t crc8_dvb_s2_calc(uint8_t crc, const uint8_t input) {
  return crc8_calc(DVB_S2_POLY, crc, input);
}

uint8_t crc8_dvb_s2_data(uint8_t crc, const uint8_t *data, const uint32_t size) {
  for (uint32_t i = 0; i < size; i++) {
    crc = crc8_dvb_s2_calc(crc, data[i]);
  }
  return crc;
}