#include "vtx.h"

#include <string.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "driver/vtx/msp.h"
#include "io/msp.h"

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

uint8_t msp_vtx_detected = 0;
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

vtx_detect_status_t vtx_msp_update(vtx_settings_t *actual) {
  if (vtx_connect_tries > MSP_VTX_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }

  const vtx_update_result_t result = serial_msp_vtx_update();
  switch (result) {
  case VTX_ERROR:
    vtx_connect_tries++;
  case VTX_WAIT:
    return VTX_DETECT_WAIT;

  case VTX_SUCCESS:
  case VTX_IDLE:
    if (!msp_vtx_detected) {
      return VTX_DETECT_WAIT;
    }

    if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
      vtx_set(actual);
    }

    memcpy(&vtx_settings.power_table, &actual->power_table, sizeof(vtx_power_table_t));

    vtx_settings.detected = VTX_PROTOCOL_MSP_VTX;
    vtx_connect_tries = 0;
    return VTX_DETECT_SUCCESS;
  }

  // wait otherwise
  return VTX_DETECT_WAIT;
}

void msp_vtx_set_frequency(vtx_band_t band, vtx_channel_t channel) {
  vtx_actual.band = band;
  vtx_actual.channel = channel;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
}

void msp_vtx_set_power_level(vtx_power_level_t power) {
  vtx_actual.power_level = power;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
}

void msp_vtx_set_pit_mode(vtx_pit_mode_t pit_mode) {
  vtx_actual.pit_mode = pit_mode;
  msp_vtx_send_config_reply(msp_vtx, MSP2_MAGIC);
}