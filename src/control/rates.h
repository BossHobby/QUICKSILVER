#pragma once

#include <stdint.h>

#include "util/vector.h"

// Angle error towards a target attitude in radians, for angle_pid().
vec3_t input_angle_vector(float roll, float pitch);
vec3_t input_stick_vector(float rx_input[]);
vec3_t input_rates_calc();
float input_rate_max(uint32_t axis);
float input_throttle_calc(float throttle);
