#pragma once

#include "util/vector.h"

void pid_init();
// Refresh cached gains after editing rates or selecting a PID profile.
void pid_rates_update();
void pid_filter_update(bool reset);
void pid_calc();
void pid_reset_i();
void pid_reset_i(uint8_t axis);
const vec3_t *pid_get_ierror();
