#pragma once

#include <stdbool.h>

#include "core/profile.h"

void output_apply_mixer_rules(void);
void output_activate_count(uint8_t count);
void output_finalize_motor_values(void);
void output_write_values();
void output_write_all();
void output_stop_all();
bool output_source_configured(output_source_t source);
