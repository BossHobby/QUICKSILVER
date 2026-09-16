#pragma once

#include "util/vector.h"

void pid_init();
// Refresh cached gains after editing rates or selecting a PID profile.
void pid_rates_update();
void pid_filter_update(bool reset);
// Wings pass sport to replace the rate integrator with the hands-off hold.
void pid_calc(bool sport = false);
// Restart integral and derivative history after a pass that skipped pid_calc().
void pid_reset();
void pid_reset_i();
void pid_reset_i(uint8_t axis);
const vec3_t *pid_get_ierror();
