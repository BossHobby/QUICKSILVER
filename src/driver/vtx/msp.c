
#include "driver/vtx/msp.h"

#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "driver/osd/displayport.h"
#include "driver/serial.h"
#include "io/msp.h"
#include "rx/unified_serial.h"
#include "util/crc.h"

#ifdef USE_VTX

extern msp_t *msp_vtx;

extern uint32_t vtx_last_valid_read;
extern uint32_t vtx_last_request;

static void serial_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  if (magic == MSP2_MAGIC) {
    const uint32_t size = len + MSP2_HEADER_LEN + 1;
    uint8_t vtx_frame[size];

    vtx_frame[0] = '$';
    vtx_frame[1] = MSP2_MAGIC;
    vtx_frame[2] = direction;
    vtx_frame[3] = 0; // flag
    vtx_frame[4] = (cmd >> 0) & 0xFF;
    vtx_frame[5] = (cmd >> 8) & 0xFF;
    vtx_frame[6] = (len >> 0) & 0xFF;
    vtx_frame[7] = (len >> 8) & 0xFF;

    memcpy(vtx_frame + MSP2_HEADER_LEN, data, len);
    vtx_frame[len + MSP2_HEADER_LEN] = crc8_dvb_s2_data(0, vtx_frame + 3, len + 5);

    serial_vtx_send_data(vtx_frame, size);
  } else {
    const uint32_t size = len + MSP_HEADER_LEN + 1;
    uint8_t vtx_frame[size];

    vtx_frame[0] = '$';
    vtx_frame[1] = MSP1_MAGIC;
    vtx_frame[2] = direction;
    vtx_frame[3] = len;
    vtx_frame[4] = cmd;

    memcpy(vtx_frame + MSP_HEADER_LEN, data, len);

    uint8_t chksum = len;
    for (uint8_t i = 4; i < (size - 1); i++) {
      chksum ^= vtx_frame[i];
    }
    vtx_frame[len + MSP_HEADER_LEN] = chksum;

    serial_vtx_send_data(vtx_frame, size);
  }
}

extern msp_t displayport_msp;
extern msp_t crsf_msp;

static uint8_t msp_rx_buffer[128];
static msp_t msp = {
    .buffer = msp_rx_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = serial_msp_send,
    .device = MSP_DEVICE_VTX,
};

void serial_msp_vtx_init() {
  if (serial_displayport.config.port != SERIAL_PORT_INVALID) {
    // reuse existing msp for hdz
    msp_vtx = &displayport_msp;
    return;
  }

  if (profile.serial.smart_audio == profile.serial.rx &&
      serial_rx_detected_protcol == RX_SERIAL_PROTOCOL_CRSF) {
    msp_vtx = &crsf_msp;
    return;
  }

  const target_serial_port_t *dev = serial_get_dev(profile.serial.smart_audio);
  if (!target_serial_port_valid(dev)) {
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.smart_audio;
  config.baudrate = 9600;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = true;
  config.half_duplex_pp = false;

  serial_vtx_wait_for_ready();
  serial_init(&serial_vtx, config);

  msp_vtx = &msp;
}

vtx_update_result_t serial_msp_vtx_update() {
  if (serial_displayport.config.port != SERIAL_PORT_INVALID) {
    if (!displayport_is_ready()) {
      return VTX_WAIT;
    }
    return VTX_IDLE;
  }

  if (profile.serial.smart_audio != SERIAL_PORT_INVALID &&
      profile.serial.smart_audio == profile.serial.rx &&
      serial_rx_detected_protcol == RX_SERIAL_PROTOCOL_CRSF) {
    return VTX_IDLE;
  }

  if (!serial_vtx_is_ready()) {
    return VTX_WAIT;
  }

  static bool in_progress = false;
  static bool is_first_packet = true;

  uint8_t data = 0;
  while (serial_vtx_read_byte(&data)) {
    quic_debugf("MSP_VTX: read 0x%x %c", data, data);

    in_progress = true;

    msp_status_t status = msp_process_serial(msp_vtx, data);
    switch (status) {
    case MSP_ERROR:
    case MSP_EOF:
      break;
    case MSP_SUCCESS:
      in_progress = false;
      is_first_packet = false;
      return VTX_SUCCESS;
    }
  }

  if ((in_progress || is_first_packet) && (time_millis() - vtx_last_valid_read) > 500) {
    quic_debugf("MSP_VTX: timeout waiting for packet");
    vtx_last_valid_read = time_millis();
    return VTX_ERROR;
  }

  if (in_progress) {
    return VTX_WAIT;
  }

  return VTX_IDLE;
}

#endif