#pragma once

#include <stdint.h>

#include "drv_osd.h"

void hdzero_init();
bool hdzero_is_ready();
void hdzero_intro();

uint8_t hdzero_clear_async();
osd_system_t hdzero_check_system();

void hdzero_txn_start(osd_transaction_t *txn, uint8_t attr, uint8_t x, uint8_t y);
void hdzero_txn_write_char(osd_transaction_t *txn, const char val);
void hdzero_txn_write_data(osd_transaction_t *txn, const uint8_t *buffer, uint8_t size);
void hdzero_txn_submit(osd_transaction_t *txn);