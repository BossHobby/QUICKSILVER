#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

static bool msp_confirm = false;

static uint8_t msp_length = 0;
static uint8_t *msp_buffer;

static uint8_t msp_bytes_per_call = 0;

static uint8_t current_package = 1;
static uint8_t current_offset = 0;

static bool finished_data = false;

bool elrs_get_msp_confirm() {
  return msp_confirm;
}

bool elrs_msp_finished_data() {
  return finished_data;
}

void elrs_msp_restart() {
  if (!finished_data) {
    return;
  }

  current_package = 1;
  current_offset = 0;
  finished_data = false;
}

void elrs_setup_msp(const uint8_t max_length, uint8_t *buffer, const uint8_t bytes_per_call) {
  msp_length = max_length;
  msp_buffer = buffer;
  msp_bytes_per_call = bytes_per_call;

  current_package = 1;
  current_offset = 0;
  finished_data = false;
}

void elrs_receive_msp(const uint8_t package_index, const uint8_t *data) {
  if (package_index == 0 && current_package > 1) {
    finished_data = true;
    msp_confirm = !msp_confirm;
    return;
  }

  if (package_index == ELRS_MSP_MAX_PACKAGES) {
    msp_confirm = !msp_confirm;
    current_package = 1;
    current_offset = 0;
    finished_data = false;
    return;
  }

  if (finished_data) {
    return;
  }

  if (package_index == current_package) {
    for (uint8_t i = 0; i < msp_bytes_per_call; i++) {
      msp_buffer[current_offset++] = *(data + i);
    }

    current_package++;
    msp_confirm = !msp_confirm;
  }
}

#endif