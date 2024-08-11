#include "vtx.h"

#include <math.h>
#include <string.h>

#include "driver/serial.h"
#include "driver/vtx/sa.h"
#include "util/util.h"

#ifdef USE_VTX

#define SMART_AUDIO_DETECT_TRIES 30

extern uint8_t vtx_connect_tries;

uint8_t smart_audio_detected = 0;
extern smart_audio_settings_t smart_audio_settings;

static bool smart_audio_needs_update = false;

static const char smart_audio_power_level_labels[4][VTX_POWER_LABEL_LEN] = {
    "25   ",
    "200  ",
    "500  ",
    "800  ",
};

static uint32_t smart_audio_dbi_to_mw(uint16_t dbi) {
  uint16_t mw = (uint16_t)powf(10.0f, dbi / 10.0f);
  if (dbi > 14) {
    // For powers greater than 25mW round up to a multiple of 50 to match expectations
    mw = 50 * ((mw + 25) / 50);
  }
  return mw;
}

static char *i2a(char *ptr, uint32_t val) {
  const uint32_t div = val / 10;
  if (div > 0)
    ptr = i2a(ptr, div);
  *ptr = '0' + (val % 10);
  return ptr + 1;
}

static void smart_audio_write_mw(char *buf, uint32_t val) {
  char *ptr = i2a(buf, val);
  while (ptr != (buf + VTX_POWER_LABEL_LEN))
    *ptr++ = ' ';
}

vtx_detect_status_t vtx_smart_audio_update(vtx_settings_t *actual) {
  if (smart_audio_settings.version == 0 && vtx_connect_tries > SMART_AUDIO_DETECT_TRIES) {
    return VTX_DETECT_ERROR;
  }

  const vtx_update_result_t result = serial_smart_audio_update();

  if ((result == VTX_IDLE || result == VTX_ERROR) && smart_audio_settings.version == 0 && vtx_connect_tries <= SMART_AUDIO_DETECT_TRIES) {
    // no smart audio detected, try again
    serial_smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
    vtx_connect_tries++;
    return VTX_DETECT_WAIT;
  }

  if (result == VTX_SUCCESS) {
    int8_t channel_index = -1;
    if (smart_audio_settings.mode & SA_MODE_FREQUENCY) {
      channel_index = vtx_find_frequency_index(smart_audio_settings.frequency);
    } else {
      channel_index = smart_audio_settings.channel;
    }

    if (channel_index >= 0) {
      actual->band = channel_index / VTX_CHANNEL_MAX;
      actual->channel = channel_index % VTX_CHANNEL_MAX;
    }

    actual->power_table.levels = smart_audio_settings.level_count;
    memcpy(actual->power_table.values, smart_audio_settings.dac_power_levels, sizeof(smart_audio_settings.dac_power_levels));
    if (smart_audio_settings.version == 3) {
      for (uint32_t i = 0; i < smart_audio_settings.level_count; i++) {
        smart_audio_write_mw(actual->power_table.labels[i], smart_audio_dbi_to_mw(smart_audio_settings.dac_power_levels[i]));
      }
    } else {
      memcpy(actual->power_table.labels, smart_audio_power_level_labels, sizeof(smart_audio_power_level_labels));
    }

    if (smart_audio_settings.version == 2) {
      actual->power_level = min(smart_audio_settings.power, VTX_POWER_LEVEL_MAX - 1);
    } else {
      actual->power_level = vtx_power_level_index(&actual->power_table, smart_audio_settings.power);
    }

    if (smart_audio_settings.version >= 3) {
      actual->pit_mode = smart_audio_settings.mode & SA_MODE_PIT ? 1 : 0;
    } else {
      actual->pit_mode = VTX_PIT_MODE_NO_SUPPORT;
    }

    if (smart_audio_settings.version != 0 && smart_audio_detected == 0) {
      smart_audio_detected = 1;

      if (vtx_settings.magic != VTX_SETTINGS_MAGIC) {
        vtx_set(actual);
      }

      vtx_settings.detected = VTX_PROTOCOL_SMART_AUDIO;
      vtx_connect_tries = 0;
    }

    if (smart_audio_needs_update) {
      serial_smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
      smart_audio_needs_update = false;
      return VTX_DETECT_WAIT;
    }

    return VTX_DETECT_SUCCESS;
  }

  if ((result == VTX_IDLE || result == VTX_ERROR) && smart_audio_detected) {
    // we are detected and vtx is in idle, we can update stuff
    return VTX_DETECT_UPDATE;
  }

  // wait otherwise
  return VTX_DETECT_WAIT;
}

void smart_audio_set_frequency(vtx_band_t band, vtx_channel_t channel) {
#ifdef SA_USE_SET_FREQUENCY
  if (smart_audio_settings.mode & SA_MODE_FREQUENCY) {
    const uint16_t frequency = frequency_table[band][channel];
    const uint8_t payload[2] = {
        (frequency >> 8) & 0xFF,
        frequency & 0xFF,
    };
    serial_smart_audio_send_payload(SA_CMD_SET_FREQUENCY, payload, 2);
  } else
#endif
  {
    smart_audio_settings.mode &= ~SA_MODE_FREQUENCY;

    const uint8_t index = band * VTX_CHANNEL_MAX + channel;
    const uint8_t payload[1] = {index};
    serial_smart_audio_send_payload(SA_CMD_SET_CHANNEL, payload, 1);
  }
}

void smart_audio_set_power_level(vtx_power_level_t power) {
  uint8_t level = power;
  if (smart_audio_settings.version != 2) {
    level = smart_audio_settings.dac_power_levels[power];
  }
  if (smart_audio_settings.version == 3) {
    level |= 0x80;
  }
  serial_smart_audio_send_payload(SA_CMD_SET_POWER, &level, 1);
}

void smart_audio_set_pit_mode(vtx_pit_mode_t pit_mode) {
  if (smart_audio_settings.version >= 3) {
    uint8_t mode = 0x0;

    if (pit_mode == VTX_PIT_MODE_OFF) {
      mode |= 0x08;
    } else if (pit_mode == VTX_PIT_MODE_ON) {
      // out-range was dropped for VTXes with SA >= v2.1
      mode |= 0x09;
    }

    serial_smart_audio_send_payload(SA_CMD_SET_MODE, &mode, 1);
    smart_audio_needs_update = true;
  } else {
    vtx_settings.pit_mode = VTX_PIT_MODE_NO_SUPPORT;
  }
}

#endif