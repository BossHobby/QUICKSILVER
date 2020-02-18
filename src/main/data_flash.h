#pragma once

#include <cbor.h>

#include "blackbox.h"

typedef struct {
  uint32_t time;
  uint32_t entries;
} data_flash_header_t;

void data_flash_init();
void data_flash_reset();

cbor_result_t data_flash_read_backbox(const uint32_t addr, blackbox_t *b);
cbor_result_t data_flash_write_backbox(const blackbox_t *b);