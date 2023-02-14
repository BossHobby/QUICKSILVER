#include "rx/express_lrs.h"

#include <string.h>

#include "util/util.h"

#if defined(RX_EXPRESS_LRS) && (defined(USE_SX127X) || defined(USE_SX128X))

typedef enum {
  SENDER_IDLE = 0,
  SENDING,
  WAIT_UNTIL_NEXT_CONFIRM,
  RESYNC,
  RESYNC_THEN_SEND, // perform a RESYNC then go to SENDING
} elrs_tlm_sender_state_t;

typedef struct {
  elrs_tlm_sender_state_t state;
  uint8_t *data;
  uint8_t length;
  uint8_t current_offset;
  uint8_t bytes_last_payload;
  uint8_t current_package;
  bool wait_until_telemetry_confirm;
  uint16_t wait_count;
  uint16_t max_wait_count;
  uint8_t max_package_index;
} elrs_tlm_sender_t;

typedef struct {
  uint8_t *data;
  bool finished_data;
  uint8_t length;
  uint8_t current_offset;
  uint8_t current_package;
  bool telemetry_confirm;
  uint8_t max_package_index;
} elrs_tlm_receiver_t;

static elrs_tlm_sender_t sender;
static elrs_tlm_receiver_t receiver;

void elrs_tlm_receiver_reset() {
  receiver.data = NULL;
  receiver.length = 0;

  receiver.current_package = 1;
  receiver.current_offset = 0;
  receiver.telemetry_confirm = false;
}

void elrs_tlm_receiver_set_max_package_index(uint8_t max_package_index) {
  if (receiver.max_package_index != max_package_index) {
    receiver.max_package_index = max_package_index;
    elrs_tlm_receiver_reset();
  }
}

bool elrs_tlm_receiver_confirm() {
  return receiver.telemetry_confirm;
}

void elrs_tlm_receiver_set_data_to_receive(uint8_t *data_to_receive, uint8_t max_length) {
  receiver.length = max_length;
  receiver.data = data_to_receive;
  receiver.current_package = 1;
  receiver.current_offset = 0;
  receiver.finished_data = false;
}

void elrs_tlm_receiver_receive_data(uint8_t const package_index, uint8_t const *const receive_data, uint8_t data_len) {
  // Resync
  if (package_index == receiver.max_package_index) {
    receiver.telemetry_confirm = !receiver.telemetry_confirm;
    receiver.current_package = 1;
    receiver.current_offset = 0;
    receiver.finished_data = false;
    return;
  }

  if (receiver.finished_data) {
    return;
  }

  bool accept_data = false;
  if (package_index == 0 && receiver.current_package > 1) {
    // PackageIndex 0 (the final packet) can also contain data
    accept_data = true;
    receiver.finished_data = true;
  } else if (package_index == receiver.current_package) {
    accept_data = true;
    receiver.current_package++;
  }

  if (accept_data) {
    uint8_t len = min((uint8_t)(receiver.length - receiver.current_offset), data_len);
    memcpy(&receiver.data[receiver.current_offset], receive_data, len);
    receiver.current_offset += len;
    receiver.telemetry_confirm = !receiver.telemetry_confirm;
  }
}

bool elrs_tlm_receiver_has_finished_data() {
  return receiver.finished_data;
}

void elrs_tlm_receiver_unlock() {
  if (receiver.finished_data) {
    receiver.current_package = 1;
    receiver.current_offset = 0;
    receiver.finished_data = false;
  }
}

void elrs_tlm_sender_reset() {
  sender.data = NULL;
  sender.length = 0;

  sender.bytes_last_payload = 0;
  sender.current_offset = 0;
  sender.current_package = 1;
  sender.wait_until_telemetry_confirm = true;
  sender.wait_count = 0;

  // 80 corresponds to UpdateTelemetryRate(ANY, 2, 1), which is what the TX uses in boost mode
  sender.max_wait_count = 80;
  sender.state = SENDER_IDLE;
}

void elrs_tlm_sender_set_max_package_index(uint8_t max_package_index) {
  if (sender.max_package_index != max_package_index) {
    sender.max_package_index = max_package_index;
    elrs_tlm_sender_reset();
  }
}

bool elrs_tlm_sender_active() {
  return sender.state != SENDER_IDLE;
}

void elrs_tlm_sender_set_data(uint8_t *data_to_transmit, const uint8_t length_to_transmit) {
  sender.length = length_to_transmit;
  sender.data = data_to_transmit;
  sender.current_offset = 0;
  sender.current_package = 1;
  sender.wait_count = 0;
  sender.state = (sender.state == SENDER_IDLE) ? SENDING : RESYNC_THEN_SEND;
}

uint8_t elrs_tlm_sender_current_payload(uint8_t *outData, uint8_t maxLen) {
  uint8_t package_index;

  sender.bytes_last_payload = 0;
  switch (sender.state) {
  case RESYNC:
  case RESYNC_THEN_SEND:
    package_index = sender.max_package_index;
    break;
  case SENDING: {
    sender.bytes_last_payload = min((uint8_t)(sender.length - sender.current_offset), maxLen);
    // If this is the last data chunk, and there has been at least one other packet
    // skip the blank packet needed for WAIT_UNTIL_NEXT_CONFIRM
    if (sender.current_package > 1 && (sender.current_offset + sender.bytes_last_payload) >= sender.length)
      package_index = 0;
    else
      package_index = sender.current_package;

    memcpy(outData, &sender.data[sender.current_offset], sender.bytes_last_payload);
  } break;
  default:
    package_index = 0;
  }

  return package_index;
}

void elrs_tlm_sender_confirm_payload(bool telemetry_confirm_value) {
  elrs_tlm_sender_state_t next_sender_state = sender.state;

  switch (sender.state) {
  case SENDING:
    if (telemetry_confirm_value != sender.wait_until_telemetry_confirm) {
      sender.wait_count++;
      if (sender.wait_count > sender.max_wait_count) {
        sender.wait_until_telemetry_confirm = !telemetry_confirm_value;
        next_sender_state = RESYNC;
      }
      break;
    }

    sender.current_offset += sender.bytes_last_payload;
    if (sender.current_offset >= sender.length) {
      // A 0th packet is always requred so the reciver can
      // differentiate a new send from a resend, if this is
      // the first packet acked, send another, else IDLE
      if (sender.current_package == 1)
        next_sender_state = WAIT_UNTIL_NEXT_CONFIRM;
      else
        next_sender_state = SENDER_IDLE;
    }

    sender.current_package++;
    sender.wait_until_telemetry_confirm = !sender.wait_until_telemetry_confirm;
    sender.wait_count = 0;
    break;

  case RESYNC:
  case RESYNC_THEN_SEND:
  case WAIT_UNTIL_NEXT_CONFIRM:
    if (telemetry_confirm_value == sender.wait_until_telemetry_confirm) {
      next_sender_state = (sender.state == RESYNC_THEN_SEND) ? SENDING : SENDER_IDLE;
      sender.wait_until_telemetry_confirm = !telemetry_confirm_value;
    }
    // switch to resync if tx does not confirm value fast enough
    else if (sender.state == WAIT_UNTIL_NEXT_CONFIRM) {
      sender.wait_count++;
      if (sender.wait_count > sender.max_wait_count) {
        sender.wait_until_telemetry_confirm = !telemetry_confirm_value;
        next_sender_state = RESYNC;
      }
    }
    break;

  case SENDER_IDLE:
    break;
  }

  sender.state = next_sender_state;
}

/*
 * Called when the telemetry ratio or air rate changes, calculate
 * the new threshold for how many times the telemetry_confirm_value
 * can be wrong in a row before giving up and going to RESYNC
 */
void elrs_tlm_sender_update_rate(const uint16_t air_rate, const uint8_t tlm_ratio, const uint8_t tlm_burst) {
  // consipicuously unused air_rate parameter, the wait count is strictly based on number
  // of packets, not time between the telemetry packets, or a wall clock timeout
  UNUSED(air_rate);
  // The expected number of packet periods between telemetry packets
  uint32_t packs_between = tlm_ratio * (1 + tlm_burst) / tlm_burst;
  sender.max_wait_count = packs_between * ELRS_TELEMETRY_MAX_MISSED_PACKETS;
}

#endif