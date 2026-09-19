#pragma once

#include "core/failloop.h"
#include "driver/servo.h"

extern float pwm_values[MOTOR_PIN_MAX];
extern uint16_t pwm_rate;
extern failloop_t fault;

// Models the m25p16 flash device boundary (busy, erase and program behavior)
// so suites that link blackbox_device_flash resolve the driver symbols.
extern uint8_t mock_m25p16_data[2 * 65536];
void mock_m25p16_reset();
unsigned mock_m25p16_busy();
void mock_m25p16_tick();
unsigned mock_m25p16_erases();
unsigned mock_m25p16_programs();
