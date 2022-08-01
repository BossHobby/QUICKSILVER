#pragma once

#include <stdint.h>

uint8_t crc8_dvb_s2_calc(uint8_t crc, const uint8_t input);
uint8_t crc8_dvb_s2_data(uint8_t crc, const uint8_t *data, const uint32_t size);