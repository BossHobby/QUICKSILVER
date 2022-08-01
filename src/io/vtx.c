#include "io/vtx.h"

#include <stddef.h>

#include "drv_gpio.h"
#include "drv_serial.h"
#include "drv_serial_vtx_msp.h"
#include "drv_serial_vtx_sa.h"
#include "drv_serial_vtx_tramp.h"
#include "drv_time.h"
#include "flight/control.h"
#include "io/usb_configurator.h"
#include "profile.h"
#include "project.h"
#include "rx.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#if defined(FPV_PIN)
static int fpv_init = 0;
#endif

vtx_settings_t vtx_settings;
uint8_t vtx_connect_tries = 0;

static vtx_settings_t vtx_actual;

static uint32_t vtx_delay_start = 0;
static uint32_t vtx_delay_ms = 1000;

extern uint8_t smart_audio_detected;
extern smart_audio_settings_t smart_audio_settings;

vtx_detect_status_t vtx_smart_audio_update(vtx_settings_t *actual);
void smart_audio_set_frequency(vtx_band_t band, vtx_channel_t channel);
void smart_audio_set_power_level(vtx_power_level_t power);
void smart_audio_set_pit_mode(vtx_pit_mode_t pit_mode);

extern uint8_t tramp_detected;
extern tramp_settings_t tramp_settings;

vtx_detect_status_t vtx_tramp_update(vtx_settings_t *actual);
void tramp_set_frequency(vtx_band_t band, vtx_channel_t channel);
void tramp_set_power_level(vtx_power_level_t power);
void tramp_set_pit_mode(vtx_pit_mode_t pit_mode);

extern uint8_t msp_vtx_detected;

vtx_detect_status_t vtx_msp_update(vtx_settings_t *actual);
void msp_vtx_set_frequency(vtx_band_t band, vtx_channel_t channel);
void msp_vtx_set_power_level(vtx_power_level_t power);
void msp_vtx_set_pit_mode(vtx_pit_mode_t pit_mode);

const uint16_t frequency_table[VTX_BAND_MAX][VTX_CHANNEL_MAX] = {
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725}, // VTX_BAND_A
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866}, // VTX_BAND_B
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945}, // VTX_BAND_E
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880}, // VTX_BAND_F
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917}, // VTX_BAND_R
    {5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613}, // VTX_BAND_L
};

uint16_t vtx_frequency_from_channel(vtx_band_t band, vtx_channel_t channel) {
  return frequency_table[band][channel];
}

int8_t vtx_find_frequency_index(uint16_t frequency) {
  for (uint8_t band = 0; band < VTX_BAND_MAX; band++) {
    for (uint8_t channel = 0; channel < VTX_CHANNEL_MAX; channel++) {
      if (frequency == frequency_table[band][channel]) {
        return band * VTX_CHANNEL_MAX + channel;
      }
    }
  }
  return -1;
}

static vtx_detect_status_t vtx_update_protocol(vtx_protocol_t proto, vtx_settings_t *actual) {
  switch (proto) {
  case VTX_PROTOCOL_TRAMP:
    return vtx_tramp_update(actual);

  case VTX_PROTOCOL_SMART_AUDIO:
    return vtx_smart_audio_update(actual);

  case VTX_PROTOCOL_MSP_VTX:
    return vtx_msp_update(actual);

  case VTX_PROTOCOL_INVALID:
  case VTX_PROTOCOL_MAX:
    return VTX_DETECT_ERROR;
  }

  return VTX_DETECT_ERROR;
}

void vtx_init() {
  vtx_settings.detected = VTX_PROTOCOL_INVALID;
  vtx_delay_start = time_millis();

  vtx_actual.band = VTX_BAND_MAX;
  vtx_actual.channel = VTX_CHANNEL_MAX;

  vtx_actual.pit_mode = VTX_PIT_MODE_MAX;
  vtx_actual.power_level = VTX_POWER_LEVEL_MAX;
}

void vtx_update() {
#if defined(FPV_PIN)
  if (rx_aux_on(AUX_FPV_SWITCH)) {
    // fpv switch on
    if (!fpv_init && flags.rx_mode == RXMODE_NORMAL && flags.on_ground == 1) {
      fpv_init = gpio_init_fpv(flags.rx_mode);
      vtx_delay_ms = 1000;
      vtx_connect_tries = 0;
    }
    if (fpv_init) {
      gpio_pin_set(FPV_PIN);
    }
  } else {
    // fpv switch off
    if (fpv_init && flags.on_ground == 1) {
      if (flags.failsafe) {
        // do nothing = hold last state
      } else {
        gpio_pin_reset(FPV_PIN);
      }
    }
  }
#endif

  if (flags.in_air) {
    // never try to do vtx stuff in-air
    return;
  }

  if (profile.serial.smart_audio == USART_PORT_INVALID && serial_hdzero_port == USART_PORT_INVALID) {
    // no serial assigned to vtx or still in use by rx
    return;
  }

  if ((time_millis() - vtx_delay_start) < vtx_delay_ms) {
    return;
  }

  vtx_delay_ms = 0;
  vtx_delay_start = time_millis();

  if (!vtx_settings.detected) {
    static vtx_protocol_t protocol_to_check = VTX_PROTOCOL_MSP_VTX;
    static uint8_t protocol_is_init = 0;

    if (serial_hdzero_port != USART_PORT_INVALID) {
      vtx_settings.protocol = VTX_PROTOCOL_MSP_VTX;
    }

    if (vtx_settings.protocol != VTX_PROTOCOL_INVALID) {
      protocol_to_check = vtx_settings.protocol;
    }

    if (!protocol_is_init) {
      switch (protocol_to_check) {
      case VTX_PROTOCOL_TRAMP:
        serial_tramp_init();
        break;

      case VTX_PROTOCOL_SMART_AUDIO:
        serial_smart_audio_init();
        break;

      case VTX_PROTOCOL_MSP_VTX:
        serial_msp_vtx_init();
        break;

      case VTX_PROTOCOL_INVALID:
      case VTX_PROTOCOL_MAX:
        break;
      }
      protocol_is_init = 1;
      return;
    }

    const vtx_detect_status_t status = vtx_update_protocol(protocol_to_check, &vtx_actual);

    if (status == VTX_DETECT_SUCCESS) {
      // detect success, save detected proto
      vtx_settings.protocol = protocol_to_check;
    } else if (status == VTX_DETECT_ERROR) {
      vtx_connect_tries = 0;
      protocol_is_init = 0;
      vtx_delay_ms = 500;

      if (vtx_settings.protocol == VTX_PROTOCOL_INVALID) {
        // only switch protocol if we are not fixed to one
        protocol_to_check++;

        if (protocol_to_check == VTX_PROTOCOL_MAX) {
          protocol_to_check = VTX_PROTOCOL_INVALID;
        }
      }
    }
    return;
  }

  const vtx_detect_status_t status = vtx_update_protocol(vtx_settings.detected, &vtx_actual);

  if (status < VTX_DETECT_SUCCESS) {
    // we are in wait or error state, do nothing
    return;
  }

  static uint8_t frequency_tries = 0;
  if (frequency_table[vtx_actual.band][vtx_actual.channel] != frequency_table[vtx_settings.band][vtx_settings.channel]) {
    if (frequency_tries >= VTX_APPLY_TRIES) {
      // give up
      vtx_settings.band = vtx_actual.band;
      vtx_settings.channel = vtx_actual.channel;
      frequency_tries = 0;
      return;
    }

    switch (vtx_settings.detected) {
    case VTX_PROTOCOL_TRAMP:
      tramp_set_frequency(vtx_settings.band, vtx_settings.channel);
      break;

    case VTX_PROTOCOL_SMART_AUDIO:
      smart_audio_set_frequency(vtx_settings.band, vtx_settings.channel);
      break;

    case VTX_PROTOCOL_MSP_VTX:
      msp_vtx_set_frequency(vtx_settings.band, vtx_settings.channel);
      break;

    case VTX_PROTOCOL_INVALID:
    case VTX_PROTOCOL_MAX:
      break;
    }

    frequency_tries++;
    vtx_delay_ms = 10;
    return;
  } else {
    frequency_tries = 0;
  }

  static uint8_t power_level_tries = 0;
  if (vtx_actual.power_level != vtx_settings.power_level) {
    if (power_level_tries >= VTX_APPLY_TRIES) {
      // give up
      vtx_settings.power_level = vtx_actual.power_level;
      power_level_tries = 0;
      return;
    }

    switch (vtx_settings.detected) {
    case VTX_PROTOCOL_TRAMP:
      tramp_set_power_level(vtx_settings.power_level);
      break;

    case VTX_PROTOCOL_SMART_AUDIO:
      smart_audio_set_power_level(vtx_settings.power_level);
      break;

    case VTX_PROTOCOL_MSP_VTX:
      msp_vtx_set_power_level(vtx_settings.power_level);
      break;

    case VTX_PROTOCOL_INVALID:
    case VTX_PROTOCOL_MAX:
      break;
    }

    power_level_tries++;
    vtx_delay_ms = 10;
    return;
  } else {
    power_level_tries = 0;
  }

  if (!flags.usb_active &&
      profile.receiver.aux[AUX_FPV_SWITCH] <= AUX_CHANNEL_11 &&
      vtx_settings.pit_mode != VTX_PIT_MODE_NO_SUPPORT) {
    // we got a aux switch set, switch pit_mode accordingly
    if (rx_aux_on(AUX_FPV_SWITCH)) {
      vtx_settings.pit_mode = 0;
    } else {
      vtx_settings.pit_mode = 1;
    }
  }

  static uint8_t pit_mode_tries = 0;
  if (vtx_actual.pit_mode != vtx_settings.pit_mode) {
    if (pit_mode_tries >= VTX_APPLY_TRIES) {
      // give up
      vtx_settings.pit_mode = vtx_actual.pit_mode;
      pit_mode_tries = 0;
      return;
    }

    switch (vtx_settings.detected) {
    case VTX_PROTOCOL_TRAMP:
      tramp_set_pit_mode(vtx_settings.pit_mode);
      break;

    case VTX_PROTOCOL_SMART_AUDIO:
      smart_audio_set_pit_mode(vtx_settings.pit_mode);
      break;

    case VTX_PROTOCOL_MSP_VTX:
      msp_vtx_set_pit_mode(vtx_settings.pit_mode);
      break;

    case VTX_PROTOCOL_INVALID:
    case VTX_PROTOCOL_MAX:
      break;
    }

    pit_mode_tries++;
    vtx_delay_ms = 10;
  } else {
    pit_mode_tries = 0;
  }
}

void vtx_set(vtx_settings_t *vtx) {
  if (vtx->protocol != VTX_PROTOCOL_INVALID && vtx_settings.protocol != vtx->protocol) {
    // if the selected protocol was changed, restart detection
    vtx_settings.detected = VTX_PROTOCOL_INVALID;
    vtx_settings.protocol = vtx->protocol;

    smart_audio_settings.version = 0;
    smart_audio_detected = 0;
    tramp_settings.freq_min = 0;
    tramp_detected = 0;
    msp_vtx_detected = 0;
  }

  vtx_settings.magic = VTX_SETTINGS_MAGIC;

  if (vtx_settings.pit_mode != VTX_PIT_MODE_NO_SUPPORT)
    vtx_settings.pit_mode = vtx->pit_mode;

  vtx_settings.power_level = vtx->power_level < VTX_POWER_LEVEL_MAX ? vtx->power_level : 0;

  vtx_settings.band = vtx->band < VTX_BAND_MAX ? vtx->band : 0;
  vtx_settings.channel = vtx->channel < VTX_CHANNEL_MAX ? vtx->channel : 0;
}

cbor_result_t cbor_encode_vtx_settings_t(cbor_value_t *enc, const vtx_settings_t *vtx) {
  cbor_result_t res = CBOR_OK;

  CBOR_CHECK_ERROR(res = cbor_encode_map_indefinite(enc));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "magic"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint16(enc, &vtx->magic));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "protocol"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->protocol));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "detected"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->detected));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "band"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->band));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "channel"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->channel));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "pit_mode"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->pit_mode));

  CBOR_CHECK_ERROR(res = cbor_encode_str(enc, "power_level"));
  CBOR_CHECK_ERROR(res = cbor_encode_uint8(enc, &vtx->power_level));

  CBOR_CHECK_ERROR(res = cbor_encode_end_indefinite(enc));
  return res;
}

cbor_result_t cbor_decode_vtx_settings_t(cbor_value_t *dec, vtx_settings_t *vtx) {
  cbor_result_t res = CBOR_OK;

  cbor_container_t map;
  CBOR_CHECK_ERROR(res = cbor_decode_map(dec, &map));

  const uint8_t *name;
  uint32_t name_len;
  for (uint32_t i = 0; i < cbor_decode_map_size(dec, &map); i++) {
    CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &name, &name_len));

    if (buf_equal_string(name, name_len, "magic")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint16(dec, &vtx->magic));
      continue;
    }

    if (buf_equal_string(name, name_len, "protocol")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8(dec, &vtx->protocol));
      continue;
    }

    if (buf_equal_string(name, name_len, "band")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8(dec, &vtx->band));
      continue;
    }

    if (buf_equal_string(name, name_len, "channel")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8(dec, &vtx->channel));
      continue;
    }

    if (buf_equal_string(name, name_len, "pit_mode")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8(dec, &vtx->pit_mode));
      continue;
    }

    if (buf_equal_string(name, name_len, "power_level")) {
      CBOR_CHECK_ERROR(res = cbor_decode_uint8(dec, &vtx->power_level));
      continue;
    }

    CBOR_CHECK_ERROR(res = cbor_decode_skip(dec));
  }
  return res;
}
