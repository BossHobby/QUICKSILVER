#pragma once

#include <stdint.h>

uint8_t crc8_dvb_s2_calc(uint8_t crc, const uint8_t input);
uint8_t crc8_dvb_s2_data(uint8_t crc, const uint8_t *data, const uint32_t size);

// CRC-16/CCITT-FALSE (polynomial 0x1021, MSB first); start from 0xFFFF.
uint16_t crc16_ccitt_data(uint16_t crc, const uint8_t *data, const uint32_t size);
