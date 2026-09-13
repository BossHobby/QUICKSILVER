#pragma once

#include <stdint.h>

#include <cbor.h>

void scheduler_init();
uint32_t scheduler_update_loop(uint32_t elapsed_cycles);
void scheduler_run(uint32_t cycles);

void task_reset_runtime();

cbor_result_t cbor_encode_task_stats(cbor_value_t *enc);