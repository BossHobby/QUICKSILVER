#include "vtx.h"

#include <string.h>

#include "driver/serial.h"
#include "driver/serial_vtx_tramp.h"

#define TRAMP_DETECT_TRIES 5

extern uint8_t vtx_connect_tries;
extern const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX];

uint8_t tramp_detected = 0;
extern tramp_settings_t tramp_settings;

static const uint16_t tramp_power_level_values[VTX_POWER_LEVEL_MAX] = {
    25,
    100,
    200,
    300,
    400,
};

static const char tramp_power_level_labels[VTX_POWER_LEVEL_MAX][3] = {
    "25 ",
    "100",
    "200",
    "300",
    "400",
};

vtx_detect_status_t vtx_tramp_update(vtx_settings_t *actual) {
  if (tramp_settings.freq_min == 0 && vtx_connect_tries > TRAMP_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }

  vtx_update_result_t result = serial_tramp_update();

  if ((result == VTX_IDLE || result == VTX_ERROR) && tramp_settings.freq_min == 0 && vtx_connect_tries <= TRAMP_DETECT_TRIES) {
    // no tramp detected, try again
    serial_tramp_send_payload('r', 0);
    vtx_connect_tries++;
    return VTX_DETECT_WAIT;
  }

  if (result == VTX_SUCCESS) {
    if (tramp_settings.freq_min == 0) {
      // tramp reset was successful but returned empty data
      return VTX_DETECT_WAIT;
    }

    if (tramp_settings.frequency == 0) {
      serial_tramp_send_payload('v', 0);
      return VTX_DETECT_WAIT;
    }

    int8_t channel_index = vtx_find_frequency_index(tramp_settings.frequency);
    if (channel_index >= 0) {
      actual->band = channel_index / VTX_CHANNEL_MAX;
      actual->channel = channel_index % VTX_CHANNEL_MAX;
    }

    actual->power_table.levels = VTX_POWER_LEVEL_MAX;
    memcpy(actual->power_table.values, tramp_power_level_values, sizeof(tramp_power_level_values));
    memcpy(actual->power_table.labels, tramp_power_level_labels, sizeof(tramp_power_level_labels));

    actual->power_level = vtx_power_level_index(&actual->power_table, tramp_settings.power);
    actual->pit_mode = tramp_settings.pit_mode;

    // not all vtxes seem to return a non-zero value. :(
    // as its unused lets just drop it.
    // if (tramp_settings.temp == 0) {
    //   serial_tramp_send_payload('s', 0);
    //   return VTX_DETECT_WAIT;
    // }

    if (tramp_settings.freq_min != 0 && tramp_settings.frequency != 0 && tramp_detected == 0) {
      tramp_detected = 1;

      if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
        vtx_set(actual);
      }

      vtx_settings.detected = VTX_PROTOCOL_TRAMP;
      vtx_connect_tries = 0;
    }
    return VTX_DETECT_SUCCESS;
  }

  if ((result == VTX_IDLE || result == VTX_ERROR) && tramp_detected) {
    // we are detected and vtx is in idle, we can update stuff
    return VTX_DETECT_UPDATE;
  }

  // wait otherwise
  return VTX_DETECT_WAIT;
}

void tramp_set_frequency(vtx_band_t band, vtx_channel_t channel) {
  const uint16_t frequency = frequency_table[band][channel];
  serial_tramp_send_payload('F', frequency);
  tramp_settings.frequency = 0;
}

void tramp_set_power_level(vtx_power_level_t power) {
  serial_tramp_send_payload('P', tramp_power_level_values[power]);
  tramp_settings.frequency = 0;
}

void tramp_set_pit_mode(vtx_pit_mode_t pit_mode) {
  serial_tramp_send_payload('I', pit_mode == VTX_PIT_MODE_ON ? 0 : 1);
  tramp_settings.frequency = 0;
}