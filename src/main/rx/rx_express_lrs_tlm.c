#include "rx_express_lrs.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

typedef enum {
  SENDER_IDLE = 0,
  SENDING,
  WAIT_UNTIL_NEXT_CONFIRM,
  RESYNC,
  RESYNC_THEN_SEND, // perform a RESYNC then go to SENDING
} elrs_tlm_sender_state_t;

static bool msp_confirm = false;

static uint8_t msp_length = 0;
static uint8_t *msp_buffer;

static uint8_t msp_bytes_per_call = 0;

static uint8_t current_package = 1;
static uint8_t current_offset = 0;

static bool finished_data = false;

static elrs_tlm_sender_state_t sender_state;

static uint8_t *sender_data = NULL;
static uint8_t sender_length = 0;
static uint8_t sender_bytes_per_call = 1;
static uint8_t sender_current_offset;
static uint8_t sender_current_package;
static bool sender_wait_until_telemetry_confirm;
static uint16_t sender_wait_count;
static uint16_t sender_max_wait_count;

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

void elrs_receive_msp(const uint8_t package_index, const volatile uint8_t *data) {
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

void elrs_tlm_sender_reset() {
  sender_data = NULL;
  sender_bytes_per_call = 1;
  sender_current_offset = 0;
  sender_current_package = 0;
  sender_length = 0;
  sender_wait_until_telemetry_confirm = true;
  sender_wait_count = 0;

  // 80 corresponds to UpdateTelemetryRate(ANY, 2, 1), which is what the TX uses in boost mode
  sender_max_wait_count = 80;

  sender_state = SENDER_IDLE;
}

bool elrs_tlm_sender_active() {
  return sender_state != SENDER_IDLE;
}

void elrs_tlm_sender_set_data(const uint8_t bpc, uint8_t *data, const uint8_t length) {
  if (length / bpc >= ELRS_TELEMETRY_MAX_PACKAGES) {
    return;
  }

  sender_length = length;
  sender_data = data;
  sender_current_offset = 0;
  sender_current_package = 1;
  sender_wait_count = 0;
  sender_bytes_per_call = bpc;
  sender_state = (sender_state == SENDER_IDLE) ? SENDING : RESYNC_THEN_SEND;
}

void elrs_tlm_current_payload(uint8_t *package_index, uint8_t *count, uint8_t **data) {
  switch (sender_state) {
  case RESYNC:
  case RESYNC_THEN_SEND:
    *package_index = ELRS_TELEMETRY_MAX_PACKAGES;
    *count = 0;
    *data = 0;
    break;
  case SENDING:
    *data = sender_data + sender_current_offset;
    *package_index = sender_current_package;
    if (sender_bytes_per_call > 1) {
      if (sender_current_offset + sender_bytes_per_call <= sender_length) {
        *count = sender_bytes_per_call;
      } else {
        *count = sender_length - sender_current_offset;
      }
    } else {
      *count = 1;
    }
    break;
  default:
    *count = 0;
    *data = 0;
    *package_index = 0;
  }
}

void elrs_tlm_confirm_payload(const bool confirm_value) {
  elrs_tlm_sender_state_t next_sender_state = sender_state;

  switch (sender_state) {
  case SENDING:
    if (confirm_value != sender_wait_until_telemetry_confirm) {
      sender_wait_count++;
      if (sender_wait_count > sender_max_wait_count) {
        sender_wait_until_telemetry_confirm = !confirm_value;
        next_sender_state = RESYNC;
      }
      break;
    }

    sender_current_offset += sender_bytes_per_call;
    sender_current_package++;
    sender_wait_until_telemetry_confirm = !sender_wait_until_telemetry_confirm;
    sender_wait_count = 0;

    if (sender_current_offset >= sender_length) {
      next_sender_state = WAIT_UNTIL_NEXT_CONFIRM;
    }

    break;

  case RESYNC:
  case RESYNC_THEN_SEND:
  case WAIT_UNTIL_NEXT_CONFIRM:
    if (confirm_value == sender_wait_until_telemetry_confirm) {
      next_sender_state = (sender_state == RESYNC_THEN_SEND) ? SENDING : SENDER_IDLE;
      sender_wait_until_telemetry_confirm = !confirm_value;
    } else if (sender_state == WAIT_UNTIL_NEXT_CONFIRM) { // switch to resync if tx does not confirm value fast enough
      sender_wait_count++;
      if (sender_wait_count > sender_max_wait_count) {
        sender_wait_until_telemetry_confirm = !confirm_value;
        next_sender_state = RESYNC;
      }
    }

    break;

  case SENDER_IDLE:
    break;
  }

  sender_state = next_sender_state;
}

/*
 * Called when the telemetry ratio or air rate changes, calculate
 * the new threshold for how many times the telemetryConfirmValue
 * can be wrong in a row before giving up and going to RESYNC
 */
void elrs_tlm_update_rate(const uint16_t air_rate, const uint8_t tlm_ratio, const uint8_t tlm_burst) {
  // consipicuously unused air_rate parameter, the wait count is strictly based on number
  // of packets, not time between the telemetry packets, or a wall clock timeout
  UNUSED(air_rate);
  // The expected number of packet periods between telemetry packets
  uint32_t packsBetween = tlm_ratio * (1 + tlm_burst) / tlm_burst;
  sender_max_wait_count = packsBetween * ELRS_TELEMETRY_MAX_MISSED_PACKETS;
}

#endif