#pragma once

#include "driver/motor.h"

#define PWM_FREQ 32000
#define PWM_DIVIDER 1

#define PWM_TOP ((PWM_CLOCK_FREQ_HZ / PWM_FREQ) - 1)

#define MOTOR_PWM_BEEPS_ON 0.2
#define MOTOR_PWM_BEEPS_OFF 0.0