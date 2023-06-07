#pragma once

#include <stdint.h>

#include "util/util.h"

#if defined(STM32H7)
#define FLASH_WORD_SIZE 32
typedef uint64_t flash_word_t;
#else
#define FLASH_WORD_SIZE 4
typedef uint32_t flash_word_t;
#endif

#define FLASH_ALIGN(offset) MEMORY_ALIGN(offset, FLASH_WORD_SIZE)

void fmc_lock();
void fmc_unlock();

void fmc_erase();

flash_word_t fmc_read(uint32_t addr);
void fmc_read_buf(uint32_t offset, uint8_t *data, uint32_t size);

void fmc_write(uint32_t addr, flash_word_t value);
void fmc_write_buf(uint32_t addr, uint8_t *data, uint32_t size);
