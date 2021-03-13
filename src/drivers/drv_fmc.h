#pragma once

#include <stdint.h>

void fmc_lock();
void fmc_unlock();

uint8_t fmc_erase();

uint32_t fmc_read(uint32_t addr);
float fmc_read_float(uint32_t addr);

void fmc_write(uint32_t addr, uint32_t value);
void fmc_write_float(uint32_t addr, float value);
