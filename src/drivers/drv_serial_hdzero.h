#pragma once

#include <stdint.h>

#include "drv_osd.h"

void hdzero_init();
bool hdzero_is_ready();
void hdzero_intro();

uint8_t hdzero_clear_async();
uint8_t hdzero_check_system();

void hdzero_txn_submit(osd_transaction_t *txn);