#include "vtx.h"

#include <string.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "driver/vtx.h"
#include "io/msp.h"
#include "rx/unified_serial.h"
#include "util/crc.h"

#ifdef USE_VTX

#define MSP_VTX_DETECT_TRIES 5

typedef struct {
  uint8_t vtx_type;
  uint8_t band;
  uint8_t channel;
  uint8_t power;
  uint8_t pitmode;
  uint8_t freq_lsb;
  uint8_t freq_msb;
  uint8_t device_is_ready;
  uint8_t low_power_disarm;
  uint8_t pit_mode_freq_lsb;
  uint8_t pit_mode_freq_msb;
  uint8_t vtx_table_available;
  uint8_t bands;
  uint8_t channels;
  uint8_t power_levels;
} msp_vtx_config_t;

extern uint8_t vtx_connect_tries;
extern vtx_settings_t vtx_actual;

extern msp_t displayport_msp;
extern msp_t crsf_msp;

bool msp_vtx_detected = false;
msp_t *msp_vtx;

char msp_vtx_band_letters[VTX_BAND_MAX] = {'A', 'B', 'E', 'F', 'R', 'L'};
uint8_t msp_vtx_band_is_factory[VTX_BAND_MAX] = {1, 1, 1, 1, 1, 1};
char msp_vtx_band_labels[VTX_BAND_MAX][8] = {
    {'B', 'A', 'N', 'D', '_', 'A', ' ', ' '}, // A
    {'B', 'A', 'N', 'D', '_', 'B', ' ', ' '}, // B
    {'B', 'A', 'N', 'D', '_', 'E', ' ', ' '}, // E
    {'F', 'A', 'T', 'S', 'H', 'A', 'R', 'K'}, // F
    {'R', 'A', 'C', 'E', ' ', ' ', ' ', ' '}, // R
    {'R', 'A', 'C', 'E', '_', 'L', 'O', 'W'}, // L
};
uint16_t msp_vtx_frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // VTX_BAND_A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // VTX_BAND_B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // VTX_BAND_E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // VTX_BAND_F
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // VTX_BAND_R
    {5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, // VTX_BAND_L
};

extern void msp_send_reply(msp_t *msp, msp_magic_t magic, uint16_t cmd, uint8_t *data, uint32_t len);

void msp_vtx_send_config_reply(msp_t *msp, msp_magic_t magic) {
  const uint16_t freq = vtx_frequency_from_channel(vtx_actual.band, vtx_actual.channel);
  msp_vtx_config_t config = {
      .vtx_type = 0,
      .band = vtx_actual.band + 1,
      .channel = vtx_actual.channel + 1,
      .power = vtx_actual.power_level + 1,
      .pitmode = vtx_actual.pit_mode == VTX_PIT_MODE_ON ? 1 : 0,
      .freq_lsb = freq & 0xFF,
      .freq_msb = (freq >> 8),
      .device_is_ready = vtx_actual.detected,
      .low_power_disarm = 0,
      .pit_mode_freq_lsb = freq & 0xFF,
      .pit_mode_freq_msb = (freq >> 8),
      .vtx_table_available = 1,
      .bands = VTX_BAND_MAX,
      .channels = VTX_CHANNEL_MAX,
      .power_levels = vtx_actual.power_table.levels,
  };

  msp_send_reply(msp, magic, MSP_VTX_CONFIG, (uint8_t *)&config, sizeof(msp_vtx_config_t));
}

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

static uint8_t msp_rx_buffer[128];
static msp_t msp = {
    .buffer = msp_rx_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = serial_msp_send,
    .device = MSP_DEVICE_VTX,
};

static void msp_vtx_init() {
  msp_vtx_detected = false;

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

static bool msp_vtx_feed_parser() {
  if (serial_displayport.config.port != SERIAL_PORT_INVALID) {
    // handled by digital vtx
    return false;
  }
  if (profile.serial.smart_audio != SERIAL_PORT_INVALID &&
      profile.serial.smart_audio == profile.serial.rx &&
      serial_rx_detected_protcol == RX_SERIAL_PROTOCOL_CRSF) {
    // handled by telemetry
    return false;
  }
  if (!serial_vtx_is_ready()) {
    return false;
  }

  uint8_t data = 0;
  while (serial_vtx_read_byte(&data)) {
    msp_process_serial(msp_vtx, data);
  }

  // did we time out?
  return msp_vtx->buffer_offset && (time_millis() - vtx_last_valid_read) > 500;
}

static vtx_detect_status_t msp_vtx_update(vtx_settings_t *actual) {
  if (vtx_connect_tries > MSP_VTX_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }
  if (msp_vtx_feed_parser()) {
    vtx_connect_tries++;
    return VTX_DETECT_WAIT;
  }
  if (!msp_vtx_detected) {
    return VTX_DETECT_WAIT;
  }

  if (vtx_settings.detected != VTX_PROTOCOL_MSP_VTX) {
    if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
      vtx_set(actual);
    }

    vtx_settings.detected = VTX_PROTOCOL_MSP_VTX;
    vtx_connect_tries = 0;
  }
  return VTX_DETECT_SUCCESS;
}

static bool msp_vtx_set_frequency(vtx_band_t band, vtx_channel_t channel) {
  vtx_actual.band = band;
  vtx_actual.channel = channel;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
  return true;
}

static bool msp_vtx_set_power_level(vtx_power_level_t power) {
  vtx_actual.power_level = power;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
  return true;
}

static bool msp_vtx_set_pit_mode(vtx_pit_mode_t pit_mode) {
  vtx_actual.pit_mode = pit_mode;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
  return true;
}

const vtx_device_t msp_vtx_device = {
    .init = msp_vtx_init,
    .update = msp_vtx_update,
    .set_frequency = msp_vtx_set_frequency,
    .set_power_level = msp_vtx_set_power_level,
    .set_pit_mode = msp_vtx_set_pit_mode,
};

#endif