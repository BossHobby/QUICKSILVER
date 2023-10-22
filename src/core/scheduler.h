#pragma once

#include <stdint.h>

#include <cbor.h>

void scheduler_init();
void scheduler_run();

void task_reset_runtime();

cbor_result_t cbor_encode_task_stats(cbor_value_t *enc);