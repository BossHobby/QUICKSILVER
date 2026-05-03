#pragma once

#include <stdbool.h>

#include "core/profile.h"

void output_set_role(output_role_t role, float value);
void output_write_all();
void output_stop_all();
bool output_role_configured(output_role_t role);
