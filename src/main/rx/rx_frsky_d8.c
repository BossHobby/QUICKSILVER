#include "rx_frsky.h"

#include "control.h"
#include "drv_spi_cc2500.h"
#include "drv_time.h"
#include "flash.h"
#include "profile.h"
#include "usb_configurator.h"
#include "util.h"

#if defined(RX_FRSKY_D8) && defined(USE_CC2500)

//Source https://www.rcgroups.com/forums/showpost.php?p=21864861

// these will get absimal range without pa/lna
#define FRSKY_ENABLE_TELEMETRY
#define FRSKY_ENABLE_HUB_TELEMETRY

#define FRSKY_D8_CHANNEL_COUNT 8
#define FRSKY_D8_HUB_FIRST_USER_ID 0x31

#define LQI_FPS 112

typedef struct {
  uint8_t length;
  uint8_t tx_id[2];
  uint8_t counter;
  uint8_t channels[14];
  uint8_t crc[2];
} frsky_d8_frame;

extern uint8_t packet[128];
extern uint8_t protocol_state;

uint8_t frsky_extract_rssi(uint8_t rssi_raw);
uint8_t frsky_detect();
void frsky_handle_bind();

void handle_overflows();
void calibrate_channels();
void set_address(uint8_t is_bind);
uint8_t next_channel(uint8_t skip);
uint8_t packet_size();

static void frsky_d8_set_rc_data() {
  uint16_t channels[FRSKY_D8_CHANNEL_COUNT];

  channels[0] = (uint16_t)(((packet[10] & 0x0F) << 8 | packet[6]));
  channels[1] = (uint16_t)(((packet[10] & 0xF0) << 4 | packet[7]));
  channels[2] = (uint16_t)(((packet[11] & 0x0F) << 8 | packet[8]));
  channels[3] = (uint16_t)(((packet[11] & 0xF0) << 4 | packet[9]));
  channels[4] = (uint16_t)(((packet[16] & 0x0F) << 8 | packet[12]));
  channels[5] = (uint16_t)(((packet[16] & 0xF0) << 4 | packet[13]));
  channels[6] = (uint16_t)(((packet[17] & 0x0F) << 8 | packet[14]));
  channels[7] = (uint16_t)(((packet[17] & 0xF0) << 4 | packet[15]));

  // frsky input is us*1.5
  // under normal conditions this is ~1480...3020
  // when tx is set to 125% this is  ~1290...3210

  // this check terrifies me, but betaflight has something similar
  for (uint8_t i = 0; i < FRSKY_D8_CHANNEL_COUNT; i++) {
    if (channels[i] < 1200 || channels[i] > 3300) {
      return;
    }
  }

  // if we made it this far, data is ready
  flags.rx_ready = 1;
  flags.failsafe = 0;

  // AETR channel order
  state.rx.axis[0] = (channels[0] - 1500) - 750;
  state.rx.axis[1] = (channels[1] - 1500) - 750;
  state.rx.axis[2] = (channels[3] - 1500) - 750;
  state.rx.axis[3] = (channels[2] - 1500);

  for (int i = 0; i < 3; i++) {
    state.rx.axis[i] *= 1.f / 750.f;
  }
  state.rx.axis[3] *= 1.f / 1500.f;

  rx_apply_stick_calibration_scale();

  //Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (channels[4] > 2000) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (channels[5] > 2000) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (channels[6] > 2000) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (channels[7] > 2000) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = 0;
  state.aux[AUX_CHANNEL_5] = 0;
  state.aux[AUX_CHANNEL_6] = 0;
  state.aux[AUX_CHANNEL_7] = 0;
  state.aux[AUX_CHANNEL_8] = 0;
  state.aux[AUX_CHANNEL_9] = 0;
  state.aux[AUX_CHANNEL_10] = 0;
  state.aux[AUX_CHANNEL_11] = 0;

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
    state.rx_rssi = constrainf(frsky_extract_rssi(packet[18]), 0.f, 100.f);
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_spi_fps(LQI_FPS);
  }
  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL) {
    if (profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_8) {
      state.rx_rssi = constrainf(((channels[(profile.receiver.aux[AUX_RSSI] + 4)]) - 1500) * 100.f / 1500.f, 0.f, 100.f);
    }
  }
}

#ifdef FRSKY_ENABLE_HUB_TELEMETRY
static uint8_t frsky_d8_hub_encode(uint8_t *buf, uint8_t data) {
  // take care of byte stuffing
  if (data == 0x5e) {
    buf[0] = 0x5d;
    buf[1] = 0x3e;
  } else if (data == 0x5d) {
    buf[0] = 0x5d;
    buf[1] = 0x3d;
  }

  buf[0] = data;
  return 1;
}

static uint8_t frsky_d8_append_hub_telemetry(uint8_t telemetry_id, uint8_t *buf) {
  static uint8_t pid_axis = 0;
  extern vec3_t *current_pid_term_pointer();

  extern int current_pid_term;
  extern int current_pid_axis;

  uint8_t size = 0;
  buf[size++] = 0;
  buf[size++] = telemetry_id;

  if (pid_axis >= 3) {
    const int16_t term = current_pid_term;
    const int16_t axis = current_pid_axis;

    buf[size++] = 0x5E;
    buf[size++] = FRSKY_D8_HUB_FIRST_USER_ID + 0;
    buf[size++] = (uint8_t)(term & 0xff);
    buf[size++] = (uint8_t)(term >> 8);
    buf[size++] = 0x5E;
    buf[size++] = 0x5E;
    buf[size++] = FRSKY_D8_HUB_FIRST_USER_ID + 1;
    buf[size++] = (uint8_t)(axis & 0xff);
    buf[size++] = (uint8_t)(axis >> 8);
    buf[size++] = 0x5E;

    pid_axis = 0;
  } else {
    const int16_t pidk = (int16_t)(current_pid_term_pointer()->axis[pid_axis] * 1000);
    buf[size++] = 0x5E;
    buf[size++] = FRSKY_D8_HUB_FIRST_USER_ID + 2 + pid_axis;
    size += frsky_d8_hub_encode(buf + size, (uint8_t)(pidk & 0xff));
    size += frsky_d8_hub_encode(buf + size, (uint8_t)(pidk >> 8));
    buf[size++] = 0x5E;

    pid_axis++;
  }

  // write size
  buf[0] = size - 2;
  return size;
}
#endif

static uint8_t frsky_d8_handle_packet() {
  static uint32_t last_packet_received_time = 0;
  static uint32_t frame_index = 0;
  static uint32_t frames_lost = 0;
  static uint32_t max_sync_delay = 50 * FRSKY_SYNC_DELAY_MAX;

#ifdef FRSKY_ENABLE_TELEMETRY
  static uint8_t telemetry[20];
#endif

  const uint32_t current_packet_received_time = time_micros();

  uint8_t ret = 0;
  switch (protocol_state) {
  case FRSKY_STATE_STARTING:
    cc2500_enter_rxmode();
    set_address(0);
    next_channel(1);
    cc2500_strobe(CC2500_SRX);

    protocol_state = FRSKY_STATE_UPDATE;
    break;
  case FRSKY_STATE_RESUME:
    // wait for IDLE
    if ((cc2500_get_status() & (0x70)) == 0) {
      handle_overflows();

      // re-enter rx mode after telemetry
      cc2500_enter_rxmode();
      next_channel(1);
      cc2500_strobe(CC2500_SRX);
    }

    if ((time_micros() - last_packet_received_time) > FRSKY_SYNC_DELAY_MAX) {
      frame_index++;
      protocol_state = FRSKY_STATE_UPDATE;
    }
    break;
  case FRSKY_STATE_UPDATE:
    protocol_state = FRSKY_STATE_DATA;
    last_packet_received_time = current_packet_received_time;
    // fallthrough
  case FRSKY_STATE_DATA: {
    uint8_t len = packet_size();
    if (len >= 20) {
      cc2500_read_fifo(packet, 20);
      frsky_d8_frame *frame = (frsky_d8_frame *)packet;

      if (frame->length == 0x11 &&
          (frame->crc[1] & 0x80) &&
          frame->tx_id[0] == bind_storage.frsky.tx_id[0] &&
          frame->tx_id[1] == bind_storage.frsky.tx_id[1]) {

        if (frame_index >= 188) {
          frame_index = 0;
        }

        if (frame->counter != frame_index) {
          quic_debugf("FRSKY_D8: frame mismatch %d vs %d", frame->counter, frame_index);
          frame_index = frame->counter;
        }

        next_channel(1);

#ifdef FRSKY_ENABLE_TELEMETRY
        if ((frame->counter % 4) == 2) {
          const uint8_t telemetry_id = packet[4];
          telemetry[0] = 0x11; // length
          telemetry[1] = bind_storage.frsky.tx_id[0];
          telemetry[2] = bind_storage.frsky.tx_id[1];
          telemetry[3] = (uint8_t)(state.vbattfilt * 100);
          telemetry[4] = (uint8_t)(state.vbattfilt * 100);
          telemetry[5] = frsky_extract_rssi(packet[18]);
#ifdef FRSKY_ENABLE_HUB_TELEMETRY
          telemetry[6] = frsky_d8_append_hub_telemetry(telemetry_id, telemetry + 8);
#else
          telemetry[6] = 0;
#endif
          telemetry[7] = telemetry_id;

          protocol_state = FRSKY_STATE_TELEMETRY;
        } else
#else
        if ((frame->counter % 4) == 2) {
          protocol_state = FRSKY_STATE_RESUME;
        } else
#endif
        {
          cc2500_strobe(CC2500_SRX);
          protocol_state = FRSKY_STATE_UPDATE;
        }

        last_packet_received_time = current_packet_received_time;
        max_sync_delay = FRSKY_SYNC_DELAY_MAX;
        // make sure we dont read the packet a second time
        frame->crc[1] = 0x00;
        frames_lost = 0;
        frame_index++;
        ret = 1;
      } else {
        quic_debugf("FRSKY_D8: invalid frame");
      }
    } else if (len > 0) {
      quic_debugf("FRSKY_D8: short frame");
    }

    handle_overflows();

    if ((current_packet_received_time - last_packet_received_time) >= max_sync_delay) {
      if (frames_lost >= 2) {
        cc2500_switch_antenna();
      }
      if (frames_lost >= FRSKY_MAX_MISSING_FRAMES) {
        quic_debugf("FRSKY_D8: failsafe");
        max_sync_delay = 10 * FRSKY_SYNC_DELAY_MAX;
        flags.failsafe = 1;
      }

      quic_debugf("FRSKY_D8: frame lost %u=%u (%u)", frame_index, (frame_index % 4), (current_packet_received_time - last_packet_received_time));
      frames_lost++;
      state.rx_rssi = 0;

      cc2500_enter_rxmode();
      next_channel(1);
      cc2500_strobe(CC2500_SRX);
      frame_index++;
      protocol_state = FRSKY_STATE_UPDATE;
      last_packet_received_time = current_packet_received_time;
    }
    break;
  }
#ifdef FRSKY_ENABLE_TELEMETRY
  case FRSKY_STATE_TELEMETRY: {

    // telemetry has to be done ~2000us after rx
    if ((time_micros() - last_packet_received_time) >= 1500) {
      const uint8_t rssi = frsky_extract_rssi(packet[18]);

      cc2500_strobe(CC2500_SIDLE);
      if (rssi > 110) {
        cc2500_set_power(5);
      } else {
        cc2500_set_power(6);
      }
      cc2500_strobe(CC2500_SFRX);
      cc2500_enter_txmode();
      cc2500_strobe(CC2500_SIDLE);
      cc2500_write_fifo(telemetry, telemetry[0] + 1);
      protocol_state = FRSKY_STATE_RESUME;
    }
    break;
  }
#endif
  }

  return ret;
}

void rx_init() {
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    return;

  cc2500_init();

  // enable gdo0 on read
  cc2500_write_reg(CC2500_IOCFG0, 0x01);

  cc2500_write_reg(CC2500_MCSM0, 0x18);
  cc2500_write_reg(CC2500_MCSM1, 0x0C);

  // frsky d
  cc2500_write_reg(CC2500_PKTLEN, 17);     // max packet lenght of 25
  cc2500_write_reg(CC2500_PKTCTRL0, 0x05); // variable pkt lenth, enable crc
  cc2500_write_reg(CC2500_PKTCTRL1, 0x04); // only append status
  cc2500_write_reg(CC2500_PATABLE, 0xFF);  // full power

  cc2500_write_reg(CC2500_FSCTRL0, 0x00);
  cc2500_write_reg(CC2500_FSCTRL1, 0x08);

  // set base freq 2404 mhz
  cc2500_write_reg(CC2500_FREQ0, 0x27);
  cc2500_write_reg(CC2500_FREQ1, 0x76);
  cc2500_write_reg(CC2500_FREQ2, 0x5C);

  cc2500_write_reg(CC2500_MDMCFG4, 0xAA);
  cc2500_write_reg(CC2500_MDMCFG3, 0x39);
  cc2500_write_reg(CC2500_MDMCFG2, 0x11);
  cc2500_write_reg(CC2500_MDMCFG1, 0x23);
  cc2500_write_reg(CC2500_MDMCFG0, 0x7A);

  cc2500_write_reg(CC2500_DEVIATN, 0x42);

  cc2500_write_reg(CC2500_FOCCFG, 0x16);
  cc2500_write_reg(CC2500_BSCFG, 0x6C);

  cc2500_write_reg(CC2500_AGCCTRL2, 0x03);
  cc2500_write_reg(CC2500_AGCCTRL1, 0x40);
  cc2500_write_reg(CC2500_AGCCTRL0, 0x91);

  cc2500_write_reg(CC2500_FREND1, 0x56);
  cc2500_write_reg(CC2500_FREND0, 0x10);

  cc2500_write_reg(CC2500_FSCAL3, 0xA9);
  cc2500_write_reg(CC2500_FSCAL2, 0x0A);
  cc2500_write_reg(CC2500_FSCAL1, 0x00);
  cc2500_write_reg(CC2500_FSCAL0, 0x11);

  cc2500_write_reg(CC2500_TEST2, 0x88);
  cc2500_write_reg(CC2500_TEST1, 0x31);
  cc2500_write_reg(CC2500_TEST0, 0x0B);

  cc2500_write_reg(CC2500_FIFOTHR, 0x07);

  cc2500_write_reg(CC2500_ADDR, 0x00);

  calibrate_channels();
}

void rx_check() {
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
    return;

  if (protocol_state <= FRSKY_STATE_BIND_COMPLETE) {
    return frsky_handle_bind();
  }

  if (frsky_d8_handle_packet()) {
    frsky_d8_set_rc_data();
  }
}

#endif
