#pragma once

#include "util/vector.h"

void pid_init();
void pid_calc();
void pid_reset_i();
const vec3_t *pid_get_ierror();
