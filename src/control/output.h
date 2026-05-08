#pragma once

#include <stdbool.h>

#include "core/profile.h"

void output_apply_mapped_sources();
void output_write_all();
void output_stop_all();
bool output_source_configured(output_source_t source);
