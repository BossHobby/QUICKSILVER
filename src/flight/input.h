#pragma once

#include "util/vector.h"

void input_stick_vector(float rx_input[], float maxangle);
vec3_t input_rates_calc();
float input_throttle_calc(float throttle);