#pragma once

#include "util/vector.h"

vec3_t input_stick_vector(float rx_input[]);
vec3_t input_rates_calc();
float input_throttle_calc(float throttle);