#include "vtx.h"

#include <stddef.h>

#include "drv_gpio.h"
#include "drv_serial.h"
#include "drv_serial_smart_audio.h"
#include "drv_time.h"
#include "project.h"
#include "rx.h"
#include "usb_configurator.h"
#include "util.h"

// bind / normal rx mode
extern int rxmode;
// failsafe on / off
extern int failsafe;

extern int onground;
extern int lastlooptime;

#ifdef FPV_ON
static int fpv_init = 0;
#endif

vtx_settings_t vtx_settings;

#ifdef ENABLE_SMART_AUDIO
#define VTX_SET_DELAY 5000
#define SMART_AUDIO_CONNECTION_TRIES 10

extern smart_audio_settings_t smart_audio_settings;
uint8_t smart_audio_detected = 0;

const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // VTX_BAND_A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // VTX_BAND_B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // VTX_BAND_E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // VTX_BAND_F
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // VTX_BAND_R
};

int8_t find_frequency_index(uint16_t frequency) {
  for (uint8_t band = 0; band < VTX_BAND_MAX; band++) {
    for (uint8_t channel = 0; channel < VTX_CHANNEL_MAX; channel++) {
      if (frequency == frequency_table[band][channel]) {
        return band * VTX_CHANNEL_MAX + channel;
      }
    }
  }
  return -1;
}

const uint8_t smart_audio_dac_power_level[4] = {
    7,
    16,
    25,
    40,
};

int8_t smart_audio_dac_power_level_index(uint8_t dac) {
  for (uint8_t level = 0; level < VTX_POWER_LEVEL_MAX; level++) {
    if (dac == smart_audio_dac_power_level[level]) {
      return level;
    }
  }
  return -1;
}

uint8_t has_smart_audio_configured() {
  return serial_smart_audio_port == profile.serial.smart_audio && serial_smart_audio_port != USART_PORT_INVALID;
}
#else
#define VTX_SET_DELAY 0
#endif

void vtx_init() {
  vtx_settings.detected = 0;

#ifdef ENABLE_SMART_AUDIO
  if (profile.serial.smart_audio != USART_PORT_INVALID) {
    serial_smart_audio_init();
  }
#endif
}

void vtx_update() {
#ifdef FPV_ON
  if (rx_aux_on(AUX_FPV_ON)) {
    // fpv switch on
    if (!fpv_init && rxmode == RXMODE_NORMAL) {
      fpv_init = gpio_init_fpv();
    }
    if (fpv_init) {
      GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_SET);
    }
  } else {
    // fpv switch off
    if (fpv_init) {
      if (failsafe) {
        GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_SET);
      } else {
        GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_RESET);
      }
    }
  }
#endif

#ifdef ENABLE_SMART_AUDIO
  if (onground && has_smart_audio_configured()) {
    static uint8_t connect_tries = 0;
    if (smart_audio_settings.version == 0 && connect_tries < SMART_AUDIO_CONNECTION_TRIES) {
      // no smart audio detected, try again
      serial_smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
      // reset loop time
      lastlooptime = gettime();
      connect_tries++;
    } else if (smart_audio_settings.version != 0 && smart_audio_detected == 0) {

      quic_debugf("smart audio version: %d", smart_audio_settings.version);

      int8_t channel_index = -1;
      if (smart_audio_settings.mode & SA_MODE_FREQUENCY) {
        channel_index = find_frequency_index(smart_audio_settings.frequency);
      } else {
        channel_index = smart_audio_settings.channel;
      }

      if (channel_index >= 0) {
        vtx_settings.band = channel_index / VTX_CHANNEL_MAX;
        vtx_settings.channel = channel_index % VTX_CHANNEL_MAX;
      }

      if (smart_audio_settings.version >= 2) {
        vtx_settings.power_level = smart_audio_settings.power;
      } else {
        vtx_settings.power_level = smart_audio_dac_power_level_index(smart_audio_settings.power);
      }

      if (smart_audio_settings.version >= 3) {
        vtx_settings.pit_mode = smart_audio_settings.mode & SA_MODE_PIT ? 1 : 0;
      } else {
        vtx_settings.pit_mode = VTX_PIT_MODE_NO_SUPPORT;
      }

      smart_audio_detected = 1;
      vtx_settings.detected = 1;
    }
  }
#endif
}

void vtx_set(vtx_settings_t *vtx) {
  if (vtx_set_pit_mode(vtx->pit_mode))
    debug_timer_delay_us(VTX_SET_DELAY);

  if (vtx_set_power_level(vtx->power_level))
    debug_timer_delay_us(VTX_SET_DELAY);

  vtx_set_frequency(vtx->band, vtx->channel);
}

uint8_t vtx_set_frequency(vtx_band_t band, vtx_channel_t channel) {
#ifdef ENABLE_SMART_AUDIO
  if (smart_audio_detected && has_smart_audio_configured()) {
    int8_t channel_index = -1;

    if (smart_audio_settings.mode & SA_MODE_FREQUENCY) {
      const uint16_t frequency = frequency_table[band][channel];
      const uint8_t payload[2] = {
          (frequency >> 8) & 0xFF,
          frequency & 0xFF,
      };
      serial_smart_audio_send_payload(SA_CMD_SET_FREQUENCY, payload, 2);

      channel_index = find_frequency_index(smart_audio_settings.frequency);
    } else {
      const uint8_t index = band * VTX_CHANNEL_MAX + channel;
      const uint8_t payload[1] = {index};
      serial_smart_audio_send_payload(SA_CMD_SET_CHANNEL, payload, 1);

      channel_index = smart_audio_settings.channel;
    }

    if (channel_index >= 0) {
      vtx_settings.band = channel_index / VTX_CHANNEL_MAX;
      vtx_settings.channel = channel_index % VTX_CHANNEL_MAX;
    }

    return 1;
  }
#endif

  return 0;
}

uint8_t vtx_set_pit_mode(vtx_pit_mode_t pit_mode) {
#ifdef ENABLE_SMART_AUDIO
  if (smart_audio_detected && has_smart_audio_configured()) {
    if (smart_audio_settings.version >= 3) {
      uint8_t mode = 0x0;

      if (smart_audio_settings.mode & SA_MODE_OUT_RANGE_PIT) {
        mode |= 0x02;
      } else {
        // default to SA_MODE_IN_RANGE_PIT
        mode |= 0x01;
      }

      if (pit_mode == VTX_PIT_MODE_OFF) {
        mode |= 0x04;
      }

      serial_smart_audio_send_payload(SA_CMD_SET_MODE, &mode, 1);
      vtx_settings.pit_mode = smart_audio_settings.mode & SA_MODE_PIT ? 1 : 0;
      return 1;
    } else {
      vtx_settings.pit_mode = VTX_PIT_MODE_NO_SUPPORT;
    }
  }
#endif

  return 0;
}

uint8_t vtx_set_power_level(vtx_power_level_t power_level) {
#ifdef ENABLE_SMART_AUDIO
  if (smart_audio_detected && has_smart_audio_configured()) {
    uint8_t level = power_level;

    if (smart_audio_settings.version >= 2) {
      serial_smart_audio_send_payload(SA_CMD_SET_POWER, &level, 1);
      vtx_settings.power_level = smart_audio_settings.power;
    } else {
      level = smart_audio_dac_power_level[power_level];
      serial_smart_audio_send_payload(SA_CMD_SET_POWER, &level, 1);
      vtx_settings.power_level = smart_audio_dac_power_level_index(smart_audio_settings.power);
    }

    return 1;
  }
#endif

  return 0;
}

cbor_result_t cbor_encode_vtx_settings_t(cbor_value_t *enc, const vtx_settings_t *vtx) {
  cbor_result_t res = CBOR_OK;

  res = cbor_encode_map_indefinite(enc);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "detected");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint8(enc, &vtx->detected);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "band");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint8(enc, &vtx->band);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "channel");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint8(enc, &vtx->channel);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "pit_mode");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint8(enc, &vtx->pit_mode);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_str(enc, "power_level");
  if (res < CBOR_OK) {
    return res;
  }
  res = cbor_encode_uint8(enc, &vtx->power_level);
  if (res < CBOR_OK) {
    return res;
  }

  res = cbor_encode_end_indefinite(enc);
  if (res < CBOR_OK) {
    return res;
  }
  return res;
}

cbor_result_t cbor_decode_vtx_settings_t(cbor_value_t *dec, vtx_settings_t *vtx) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t map;
  res = cbor_decode_map(dec, &map);
  if (res < CBOR_OK)
    return res;

  const uint8_t *name;
  uint32_t name_len;
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    res = cbor_decode_tstr(dec, &name, &name_len);
    if (res < CBOR_OK)
      return res;

    if (buf_equal_string(name, name_len, "band")) {
      res = cbor_decode_uint8(dec, &vtx->band);
      if (res < CBOR_OK) {
        return res;
      }
      continue;
    }

    if (buf_equal_string(name, name_len, "channel")) {
      res = cbor_decode_uint8(dec, &vtx->channel);
      if (res < CBOR_OK) {
        return res;
      }
      continue;
    }

    if (buf_equal_string(name, name_len, "pit_mode")) {
      res = cbor_decode_uint8(dec, &vtx->pit_mode);
      if (res < CBOR_OK) {
        return res;
      }
      continue;
    }

    if (buf_equal_string(name, name_len, "power_level")) {
      res = cbor_decode_uint8(dec, &vtx->power_level);
      if (res < CBOR_OK) {
        return res;
      }
      continue;
    }

    res = cbor_decode_skip(dec);
    if (res < CBOR_OK)
      return res;
  }
  return res;
}