#pragma once

#include "util/vector.h"

void pid_init();
void pid_calc();
void pid_reset_i();
void pid_reset_i(uint8_t axis);
const vec3_t *pid_get_ierror();
