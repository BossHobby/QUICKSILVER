#pragma once

#include <stdint.h>

#include "util/vector.h"

vec3_t input_stick_vector(float rx_input[]);
vec3_t input_rates_calc();
float input_rate_max(uint32_t axis);
float input_throttle_calc(float throttle);
