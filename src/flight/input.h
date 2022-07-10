#pragma once

#include "util/vector.h"

void input_stick_vector(float rx_input[], float maxangle);
void input_rates_calc(vec3_t *rates);
float input_throttle_calc(float throttle);