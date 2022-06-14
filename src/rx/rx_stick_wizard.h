#pragma once

#include <stdint.h>

typedef enum {
  INACTIVE,
  CAPTURE_STICKS,
  WAIT_FOR_CONFIRM,
  CALIBRATION_CONFIRMED,
  TIMEOUT,
  CALIBRATION_SUCCESS,
  CALIBRATION_FAILED
} stick_wizard_state_t;

void rx_apply_stick_calibration_scale();
void request_stick_calibration_wizard();