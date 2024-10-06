#include "io/vtx.h"

#include <stddef.h>
#include <string.h>

#include "core/debug.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/gpio.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "flight/control.h"
#include "rx/rx.h"
#include "rx/unified_serial.h"
#include "util/cbor_helper.h"
#include "util/util.h"

#ifdef USE_VTX

vtx_settings_t vtx_settings = {
    .power_table = {
        .levels = 0,
    },
};

vtx_settings_t vtx_actual;

static uint8_t apply_tries = 0;
static uint32_t vtx_delay_start = 0;
static uint32_t vtx_delay_ms = 100;
static bool protocol_is_init = false;

static const vtx_device_t *vtx_device = NULL;
extern const vtx_device_t msp_vtx_device;
extern const vtx_device_t smart_audio_vtx_device;
extern const vtx_device_t tramp_vtx_device;

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

vtx_power_level_t vtx_power_level_index(vtx_power_table_t *power_table, uint16_t power) {
  for (uint8_t level = 0; level < VTX_POWER_LEVEL_MAX; level++) {
    if (power >= power_table->values[level] && power <= power_table->values[level]) {
      return level;
    }
  }
  return VTX_POWER_LEVEL_1;
}

void vtx_init() {
  vtx_settings.detected = VTX_PROTOCOL_INVALID;
  vtx_delay_start = time_millis();

  vtx_actual.band = VTX_BAND_MAX;
  vtx_actual.channel = VTX_CHANNEL_MAX;

  vtx_actual.pit_mode = VTX_PIT_MODE_MAX;
  vtx_actual.power_level = VTX_POWER_LEVEL_MAX;
}

static void vtx_update_fpv_pin() {
  if (target.fpv != PIN_NONE) {
    static bool fpv_init = false;
    if (rx_aux_on(AUX_FPV_SWITCH)) {
      // fpv switch on
      if (!fpv_init && flags.rx_mode == RXMODE_NORMAL && flags.on_ground == 1) {
        fpv_init = gpio_init_fpv(flags.rx_mode);
        vtx_delay_ms = 100;
      }
      if (fpv_init) {
        gpio_pin_set(target.fpv);
      }
    } else if (fpv_init && flags.on_ground == 1 && !flags.failsafe) {
      // fpv switch off
      gpio_pin_reset(target.fpv);
    }
  }
}

static void vtx_init_protocol() {
  switch (vtx_settings.protocol) {
  case VTX_PROTOCOL_TRAMP:
    vtx_device = &tramp_vtx_device;
    break;

  case VTX_PROTOCOL_SMART_AUDIO:
    vtx_device = &smart_audio_vtx_device;
    break;

  case VTX_PROTOCOL_MSP_VTX:
    vtx_device = &msp_vtx_device;
    break;

  case VTX_PROTOCOL_INVALID:
  case VTX_PROTOCOL_MAX:
    vtx_device = NULL;
    return;
  }
  vtx_device->init();
}

static bool vtx_update_frequency() {
  if (frequency_table[vtx_actual.band][vtx_actual.channel] == frequency_table[vtx_settings.band][vtx_settings.channel]) {
    apply_tries = 0;
    return true;
  }

  if (apply_tries >= VTX_APPLY_TRIES) {
    // give up
    vtx_settings.band = vtx_actual.band;
    vtx_settings.channel = vtx_actual.channel;
    apply_tries = 0;
    return true;
  }

  if (vtx_device->set_frequency(vtx_settings.band, vtx_settings.channel)) {
    vtx_delay_ms = 10;
    apply_tries++;
  }
  return false;
}

static bool vtx_update_powerlevel() {
  if (vtx_actual.power_level == vtx_settings.power_level) {
    apply_tries = 0;
    return true;
  }

  if (apply_tries >= VTX_APPLY_TRIES) {
    // give up
    vtx_settings.power_level = vtx_actual.power_level;
    apply_tries = 0;
    return true;
  }

  if (vtx_device->set_power_level(vtx_settings.power_level)) {
    vtx_delay_ms = 10;
    apply_tries++;
  }
  return false;
}

static bool vtx_update_pitmode() {
  if (vtx_actual.pit_mode == vtx_settings.pit_mode) {
    apply_tries = 0;
    return true;
  }

  if (apply_tries >= VTX_APPLY_TRIES) {
    // give up
    vtx_settings.pit_mode = vtx_actual.pit_mode;
    apply_tries = 0;
    return true;
  }

  if (vtx_device->set_pit_mode(vtx_settings.pit_mode)) {
    vtx_delay_ms = 10;
    apply_tries++;
  }
  return false;
}

void vtx_update() {
  vtx_update_fpv_pin();

  if (flags.in_air)
    return;

  if (profile.serial.smart_audio == SERIAL_PORT_INVALID &&
      serial_displayport.config.port == SERIAL_PORT_INVALID)
    return;

  if (profile.serial.smart_audio != SERIAL_PORT_INVALID &&
      profile.serial.smart_audio == profile.serial.rx &&
      serial_rx_detected_protcol != RX_SERIAL_PROTOCOL_CRSF)
    return;

  if ((time_millis() - vtx_delay_start) < vtx_delay_ms)
    return;

  vtx_delay_ms = 0;
  vtx_delay_start = time_millis();

  // auto-select MSP_VTX for displayport or CRSF setups
  if (vtx_settings.protocol == VTX_PROTOCOL_INVALID) {
    if (serial_displayport.config.port != SERIAL_PORT_INVALID) {
      vtx_settings.protocol = VTX_PROTOCOL_MSP_VTX;
    } else if (profile.serial.smart_audio != SERIAL_PORT_INVALID &&
               profile.serial.smart_audio == profile.serial.rx &&
               serial_rx_detected_protcol == RX_SERIAL_PROTOCOL_CRSF) {
      vtx_settings.protocol = VTX_PROTOCOL_MSP_VTX;
    } else {
      return;
    }
  }

  if (!protocol_is_init) {
    vtx_init_protocol();
    protocol_is_init = true;
    return;
  }

  if (vtx_device == NULL)
    return;

  const vtx_detect_status_t status = vtx_device->update(&vtx_actual);
  if (status < VTX_DETECT_SUCCESS)
    return;

  if (profile.receiver.aux[AUX_FPV_SWITCH] <= AUX_CHANNEL_11 && vtx_settings.pit_mode != VTX_PIT_MODE_NO_SUPPORT)
    vtx_settings.pit_mode = rx_aux_on(AUX_FPV_SWITCH) ? 0 : 1;

  if (!vtx_update_pitmode() || !vtx_update_frequency() || !vtx_update_powerlevel())
    return;
}

void vtx_set(vtx_settings_t *vtx) {
  if (vtx->protocol != VTX_PROTOCOL_INVALID && vtx_settings.protocol != vtx->protocol) {
    // if the selected protocol was changed, restart detection
    vtx_settings.detected = VTX_PROTOCOL_INVALID;
    vtx_settings.protocol = vtx->protocol;
    vtx_settings.magic = 0xFFFF;
    vtx_settings.power_table.levels = 1;
    protocol_is_init = false;
    vtx_device = NULL;
  } else {
    vtx_settings.magic = VTX_SETTINGS_MAGIC;
    memcpy(&vtx_settings.power_table, &vtx->power_table, sizeof(vtx_power_table_t));
  }

  if (vtx_settings.pit_mode != VTX_PIT_MODE_NO_SUPPORT)
    vtx_settings.pit_mode = vtx->pit_mode;

  vtx_settings.power_level = vtx->power_level < vtx_settings.power_table.levels ? vtx->power_level : (vtx_settings.power_table.levels - 1);

  vtx_settings.band = vtx->band < VTX_BAND_MAX ? vtx->band : 0;
  vtx_settings.channel = vtx->channel < VTX_CHANNEL_MAX ? vtx->channel : 0;

  apply_tries = 0;
}

#define MEMBER CBOR_ENCODE_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define TSTR_ARRAY_MEMBER CBOR_ENCODE_TSTR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(vtx_power_table_t)
VTX_POWER_TABLE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(vtx_settings_t)
VTX_SETTINGS_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef ARRAY_MEMBER
#undef TSTR_ARRAY_MEMBER

#define MEMBER CBOR_DECODE_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define TSTR_ARRAY_MEMBER CBOR_DECODE_TSTR_ARRAY_MEMBER

CBOR_START_STRUCT_DECODER(vtx_power_table_t)
VTX_POWER_TABLE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(vtx_settings_t)
VTX_SETTINGS_MEMBERS
CBOR_END_STRUCT_DECODER()

#undef MEMBER
#undef ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

#else
void vtx_init() {}
void vtx_update() {}
#endif