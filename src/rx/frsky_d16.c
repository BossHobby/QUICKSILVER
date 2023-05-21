#include "rx/frsky.h"

#include <string.h>

#include "core/debug.h"
#include "core/flash.h"
#include "core/profile.h"
#include "driver/spi_cc2500.h"
#include "driver/time.h"
#include "flight/control.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#if defined(RX_FRSKY)

// Source https://www.rcgroups.com/forums/showpost.php?p=21864861

#define FRSKY_ENABLE_TELEMETRY
#define LQI_FPS 112

#define FRSKY_D16_CHANNEL_COUNT 16
#define FRSKY_D16_TELEMETRY_SEQUENCE_LENGTH 4

#define FRSKY_D16_PACKET_DELAY 5300

#define FRSKY_D16_FCC_PACKET_LENGTH 32
#define FRSKY_D16_FCC_TELEMETRY_DELAY 400

#define FRSKY_D16_LBT_PACKET_LENGTH 35
#define FRSKY_D16_LBT_TELEMETRY_DELAY 1400

#define FRSKY_D16_PACKET_LENGTH (profile.receiver.protocol == RX_PROTOCOL_FRSKY_D16_FCC ? FRSKY_D16_FCC_PACKET_LENGTH : FRSKY_D16_LBT_PACKET_LENGTH)
#define FRSKY_D16_TELEMETRY_DELAY (profile.receiver.protocol == RX_PROTOCOL_FRSKY_D16_FCC ? FRSKY_D16_FCC_TELEMETRY_DELAY : FRSKY_D16_LBT_TELEMETRY_DELAY)

const uint16_t crc_table[] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};

typedef union {
  struct {
    uint8_t packet_id : 2;
    uint8_t _unused : 1;
    uint8_t init_request : 1;
    uint8_t ack_id : 2;
    uint8_t retransmit : 1;
    uint8_t init_response : 1;
  } __attribute__((__packed__)) flag;
  uint8_t raw;
} __attribute__((__packed__)) frsky_d16_telemetry_flag_t;

typedef struct {
  uint8_t length;
  uint8_t tx_id[2];
  uint8_t constant;
  uint8_t counter;
  uint8_t chanskip;
  uint8_t rx_num;
  uint8_t flags;
  uint8_t _unused;
  uint8_t channels[12];
  frsky_d16_telemetry_flag_t telemetry;
} __attribute__((__packed__)) frsky_d16_frame_header;

extern uint8_t protocol_state;
extern uint8_t list_length;

uint8_t frsky_extract_rssi(uint8_t rssi_raw);
uint8_t frsky_detect();
void frsky_handle_bind();

void handle_overflows();
void calibrate_channels();
void set_address(uint8_t is_bind);
uint8_t next_channel(uint8_t skip);

static uint16_t frsky_d16_crc(const uint8_t *data, uint8_t len) {
  uint16_t crc = 0;
  for (unsigned i = 0; i < len; i++) {
    crc = (crc << 8) ^ (crc_table[((uint8_t)(crc >> 8) ^ *data++) & 0xFF]);
  }
  return crc;
}

static void frsky_d16_set_rc_data() {
  uint16_t temp_channels[8] = {
      (uint16_t)((rx_spi_packet[10] << 8) & 0xF00) | rx_spi_packet[9],
      (uint16_t)((rx_spi_packet[11] << 4) & 0xFF0) | (rx_spi_packet[10] >> 4),
      (uint16_t)((rx_spi_packet[13] << 8) & 0xF00) | rx_spi_packet[12],
      (uint16_t)((rx_spi_packet[14] << 4) & 0xFF0) | (rx_spi_packet[13] >> 4),
      (uint16_t)((rx_spi_packet[16] << 8) & 0xF00) | rx_spi_packet[15],
      (uint16_t)((rx_spi_packet[17] << 4) & 0xFF0) | (rx_spi_packet[16] >> 4),
      (uint16_t)((rx_spi_packet[19] << 8) & 0xF00) | rx_spi_packet[18],
      (uint16_t)((rx_spi_packet[20] << 4) & 0xFF0) | (rx_spi_packet[19] >> 4),
  };

  static uint16_t channels[FRSKY_D16_CHANNEL_COUNT] = {0};
  for (uint8_t i = 0; i < 8; i++) {
    if (rx_spi_packet[7] != (0x10 + (i << 1))) {
      const uint8_t is_shifted = ((temp_channels[i] & 0x800) > 0) ? 1 : 0;
      const uint16_t channel_value = temp_channels[i] & 0x7FF;
      channels[is_shifted ? i + 8 : i] = channel_value - 64;
    }
  }

  // if we made it this far, data is ready
  flags.rx_ready = 1;
  flags.failsafe = 0;

  // AETR channel order
  const float rc_channels[4] = {
      (channels[0] - 960.0f) * (1.f / 760.f),
      (channels[1] - 960.0f) * (1.f / 760.f),
      (channels[2] - 960.0f) * (1.f / 760.f),
      (channels[3] - 960.0f) * (1.f / 760.f),
  };
  rx_map_channels(rc_channels);

  // Here we have the AUX channels Silverware supports
  state.aux[AUX_CHANNEL_0] = (channels[4] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_1] = (channels[5] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_2] = (channels[6] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_3] = (channels[7] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_4] = (channels[8] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_5] = (channels[9] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_6] = (channels[10] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_7] = (channels[11] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_8] = (channels[12] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_9] = (channels[13] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_10] = (channels[14] > 1023) ? 1 : 0;
  state.aux[AUX_CHANNEL_11] = (channels[15] > 1023) ? 1 : 0;

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_CHANNEL && profile.receiver.aux[AUX_RSSI] <= AUX_CHANNEL_11) {
    rx_lqi_update_direct(((channels[(profile.receiver.aux[AUX_RSSI] + 4)]) - 200) * 100.f / 1520.f);
  }
}

uint8_t frsky_d16_is_valid_packet(uint8_t *packet) {
  frsky_d16_frame_header *header = (frsky_d16_frame_header *)packet;

  const uint16_t remote_crc = ((uint16_t)packet[FRSKY_D16_PACKET_LENGTH - 4]) << 8 | ((uint16_t)packet[FRSKY_D16_PACKET_LENGTH - 3]);
  const uint16_t local_crc = frsky_d16_crc(&packet[3], FRSKY_D16_PACKET_LENGTH - 7);

  if (remote_crc != local_crc) {
    quic_debugf("FRSKY_D16: invalid crc remote 0x%x vs local 0x%x", remote_crc, local_crc);
    return 0;
  }

  return header->length == (FRSKY_D16_PACKET_LENGTH - 3) &&
         header->tx_id[0] == bind_storage.frsky.tx_id[0] &&
         header->tx_id[1] == bind_storage.frsky.tx_id[1] &&
         (bind_storage.frsky.rx_num == 0 || header->rx_num == 0 || bind_storage.frsky.rx_num == header->rx_num);
}

#define SMART_PORT_DATA_SIZE 128
static uint8_t smart_port_data[SMART_PORT_DATA_SIZE];
static ring_buffer_t smart_port_buffer = {
    .buffer = smart_port_data,
    .head = 0,
    .tail = 0,
    .size = SMART_PORT_DATA_SIZE,
};

void frsky_d16_write_telemetry(smart_port_payload_t *payload) {
  if (ring_buffer_free(&smart_port_buffer) < (sizeof(smart_port_payload_t) + 2)) {
    return;
  }

  ring_buffer_write(&smart_port_buffer, FSSP_START_STOP);
  ring_buffer_write(&smart_port_buffer, FSSP_SENSOR_ID1 & 0x1f);

  const uint8_t *ptr = (uint8_t *)payload;
  for (uint32_t i = 0; i < sizeof(smart_port_payload_t); i++) {
    const uint8_t c = ptr[i];
    if (c == FSSP_DLE || c == FSSP_START_STOP) {
      ring_buffer_write(&smart_port_buffer, FSSP_DLE);
      ring_buffer_write(&smart_port_buffer, c ^ FSSP_DLE_XOR);
    } else {
      ring_buffer_write(&smart_port_buffer, c);
    }
  }
}

static uint8_t frsky_d16_append_telemetry(uint8_t *buf) {
  uint8_t size = 0;
  for (; size < 5; size++) {
    if (ring_buffer_read(&smart_port_buffer, &buf[size]) == 0) {
      break;
    }
  }
  return size;
}

static void frsky_d16_build_telemetry(uint8_t *telemetry) {
  const uint8_t rssi = frsky_extract_rssi(rx_spi_packet[FRSKY_D16_PACKET_LENGTH - 2]);

  static uint8_t even_odd = 0;
  static uint8_t local_packet_id = 0;

  telemetry[0] = 0x0E; // length
  telemetry[1] = bind_storage.frsky.tx_id[0];
  telemetry[2] = bind_storage.frsky.tx_id[1];
  telemetry[3] = rx_spi_packet[3];
  if (even_odd) {
    telemetry[4] = rssi | 0x80;
  } else {
    telemetry[4] = (uint8_t)(state.vbat_filtered * 10) & 0x7f;
  }
  even_odd = even_odd == 1 ? 0 : 1;

  frsky_d16_telemetry_flag_t *in_flag = (frsky_d16_telemetry_flag_t *)&rx_spi_packet[21];
  frsky_d16_telemetry_flag_t *out_flag = (frsky_d16_telemetry_flag_t *)&telemetry[5];
  if (in_flag->flag.init_request) {
    out_flag->raw = 0;
    out_flag->flag.init_request = 1;
    out_flag->flag.init_response = 1;

    local_packet_id = 0;
  } else {
    const uint8_t local_ack_id = in_flag->flag.ack_id;
    if (in_flag->flag.retransmit) {
      out_flag->raw = 0;
      out_flag->flag.ack_id = local_ack_id;
      out_flag->flag.packet_id = in_flag->flag.ack_id;

    } else if (local_packet_id != (local_ack_id + 1) % FRSKY_D16_TELEMETRY_SEQUENCE_LENGTH) {
      out_flag->raw = 0;
      out_flag->flag.ack_id = in_flag->flag.packet_id;
      out_flag->flag.packet_id = local_packet_id;

      telemetry[6] = frsky_d16_append_telemetry(&telemetry[7]);

      local_packet_id = (local_packet_id + 1) % FRSKY_D16_TELEMETRY_SEQUENCE_LENGTH;
    }
  }

  uint16_t crc = frsky_d16_crc(&telemetry[3], 10);
  telemetry[13] = crc >> 8;
  telemetry[14] = crc;
}

static uint8_t frsky_d16_handle_packet() {
  static uint8_t frame_had_packet = 0;
  static uint32_t frames_lost = 0;

  static uint8_t send_telemetry = 0;
  static uint8_t channel_skip = 1;
  static uint8_t channels_to_skip = 1;

  static uint32_t max_sync_delay = 50 * FRSKY_SYNC_DELAY_MAX;
  static uint32_t rx_delay = 0;

  static uint32_t last_packet_received_time = 0;

  static uint8_t telemetry[20];

  uint8_t ret = 0;
  switch (protocol_state) {
  case FRSKY_STATE_STARTING:
    cc2500_enter_rxmode();
    set_address(0);
    cc2500_write_reg(CC2500_FIFOTHR, 0x9);
    next_channel(1);
    cc2500_strobe(CC2500_SRX);

    protocol_state = FRSKY_STATE_UPDATE;
    break;
  case FRSKY_STATE_UPDATE:
    frame_had_packet = 0;
    protocol_state = FRSKY_STATE_DATA;
    rx_delay = FRSKY_D16_PACKET_DELAY;
    last_packet_received_time = time_micros();
    // fallthrough
  case FRSKY_STATE_DATA: {
    uint8_t len = cc2500_packet_size();
    if (frame_had_packet == 0 && len >= FRSKY_D16_PACKET_LENGTH) {
      cc2500_read_fifo((uint8_t *)rx_spi_packet, FRSKY_D16_PACKET_LENGTH);
      frsky_d16_frame_header *header = (frsky_d16_frame_header *)rx_spi_packet;

      if (frsky_d16_is_valid_packet((uint8_t *)rx_spi_packet)) {
        last_packet_received_time = time_micros();
        max_sync_delay = FRSKY_SYNC_DELAY_MAX;

        if (channel_skip) {
          channels_to_skip = header->chanskip << 2;
          if (header->counter >= list_length) {
            if (header->counter < (64 + list_length)) {
              channels_to_skip += 1;
            } else if (header->counter < (128 + list_length)) {
              channels_to_skip += 2;
            } else if (header->counter < (192 + list_length)) {
              channels_to_skip += 3;
            }
          }
          send_telemetry = 1;
          channel_skip = 0;
        }

        rx_lqi_got_packet();
        if (profile.receiver.lqi_source == RX_LQI_SOURCE_DIRECT) {
          rx_lqi_update_direct(frsky_extract_rssi(rx_spi_packet[FRSKY_D16_PACKET_LENGTH - 2]));
        }

        frame_had_packet = 1;
        rx_delay = 0;
        frames_lost = 0;
        ret = 1;
      } else {
        quic_debugf("FRSKY_D16: invalid frame");
      }
    } else if (len > 0) {
      quic_debugf("FRSKY_D16: invalid size %d", len);
    }

    handle_overflows();

    if (send_telemetry == 1 && (time_micros() - last_packet_received_time) >= rx_delay) {
      frsky_d16_build_telemetry(telemetry);
      protocol_state = FRSKY_STATE_TELEMETRY;
    }

    if ((time_micros() - last_packet_received_time) >= max_sync_delay) {
      if (frames_lost >= FRSKY_MAX_MISSING_FRAMES) {
        max_sync_delay = 50 * FRSKY_SYNC_DELAY_MAX;
        flags.failsafe = 1;
        state.rx_rssi = 0;
        channel_skip = 1;
        send_telemetry = 0;
        protocol_state = FRSKY_STATE_UPDATE;
        break;
      }

      if (frames_lost >= 2) {
        cc2500_switch_antenna();
      }

      if (frame_had_packet == 0) {
        rx_lqi_lost_packet();
        state.rx_rssi = 0;
        frames_lost++;
      }

      // make sure we are in rx mode
      cc2500_enter_rxmode();
      next_channel(1);
      cc2500_strobe(CC2500_SRX);
      protocol_state = FRSKY_STATE_UPDATE;
    }
    break;
  }
  case FRSKY_STATE_TELEMETRY:
    // move to idle state
    if ((cc2500_get_status() & (0x70)) != 0) {
      const uint8_t rssi = frsky_extract_rssi(rx_spi_packet[FRSKY_D16_PACKET_LENGTH - 2]);

      rx_lqi_got_packet();

      cc2500_strobe(CC2500_SIDLE);
      if (rssi > 110) {
        cc2500_set_power(5);
      } else {
        cc2500_set_power(6);
      }
      cc2500_strobe(CC2500_SFRX);
      cc2500_enter_txmode();
      cc2500_strobe(CC2500_SIDLE);
    }
    if ((time_micros() - last_packet_received_time) >= (rx_delay + FRSKY_D16_TELEMETRY_DELAY)) {
      cc2500_write_fifo(telemetry, telemetry[0] + 1);

      protocol_state = FRSKY_STATE_RESUME;
    }
    break;
  case FRSKY_STATE_RESUME:
    if ((cc2500_get_status() & (0x70)) == 0) {
      cc2500_enter_rxmode();
      next_channel(channels_to_skip);
      cc2500_strobe(CC2500_SRX);
    }
    if ((time_micros() - last_packet_received_time) >= (rx_delay + 3700)) {
      if (frames_lost >= 2) {
        cc2500_switch_antenna();
      }

      if (frames_lost >= FRSKY_MAX_MISSING_FRAMES) {
        max_sync_delay = 50 * FRSKY_SYNC_DELAY_MAX;
        flags.failsafe = 1;
        state.rx_rssi = 0;
        channel_skip = 1;
        send_telemetry = 0;
        protocol_state = FRSKY_STATE_UPDATE;
        break;
      }

      if (frame_had_packet == 0) {
        frames_lost++;
      }

      last_packet_received_time = time_micros();
      rx_delay = FRSKY_D16_PACKET_DELAY;
      frame_had_packet = 0;
      protocol_state = FRSKY_STATE_DATA;
    }
    break;
  }

  return ret;
}

void rx_frsky_d16_init() {
  if (!frsky_init()) {
    return;
  }

  // enable gdo0 on read
  cc2500_write_reg(CC2500_IOCFG0, 0x01);

  cc2500_write_reg(CC2500_MCSM0, 0x18);
  cc2500_write_reg(CC2500_MCSM1, 0x0C);

  cc2500_write_reg(CC2500_PKTCTRL1, 0x04); // only append status
  cc2500_write_reg(CC2500_PATABLE, 0xFF);  // full power

  cc2500_write_reg(CC2500_FSCTRL0, 0x00);

  // set base freq 2404 mhz
  cc2500_write_reg(CC2500_FREQ2, 0x5C);

  cc2500_write_reg(CC2500_MDMCFG1, 0x23);
  cc2500_write_reg(CC2500_MDMCFG0, 0x7A);

  if (profile.receiver.protocol == RX_PROTOCOL_FRSKY_D16_FCC) {
    cc2500_write_reg(CC2500_FREQ1, 0x76);
    cc2500_write_reg(CC2500_FREQ0, 0x27);
    cc2500_write_reg(CC2500_PKTLEN, 0x1E);
    cc2500_write_reg(CC2500_PKTCTRL0, 0x01);
    cc2500_write_reg(CC2500_FSCTRL1, 0x0A);
    cc2500_write_reg(CC2500_MDMCFG4, 0x7B);
    cc2500_write_reg(CC2500_MDMCFG3, 0x61);
    cc2500_write_reg(CC2500_MDMCFG2, 0x13);
    cc2500_write_reg(CC2500_DEVIATN, 0x51);
  } else {
    cc2500_write_reg(CC2500_FREQ1, 0x80);
    cc2500_write_reg(CC2500_FREQ0, 0x00);
    cc2500_write_reg(CC2500_PKTLEN, 0x23);
    cc2500_write_reg(CC2500_PKTCTRL0, 0x01);
    cc2500_write_reg(CC2500_FSCTRL1, 0x08);
    cc2500_write_reg(CC2500_MDMCFG4, 0x7B);
    cc2500_write_reg(CC2500_MDMCFG3, 0xF8);
    cc2500_write_reg(CC2500_MDMCFG2, 0x03);
    cc2500_write_reg(CC2500_DEVIATN, 0x53);
  }

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

  cc2500_write_reg(CC2500_FSTEST, 0x59);

  cc2500_write_reg(CC2500_TEST2, 0x88);
  cc2500_write_reg(CC2500_TEST1, 0x31);
  cc2500_write_reg(CC2500_TEST0, 0x0B);

  cc2500_write_reg(CC2500_FIFOTHR, 0x07);

  cc2500_write_reg(CC2500_ADDR, 0x00);

  calibrate_channels();

  list_length = 47;
}

bool rx_frsky_d16_check() {
  bool channels_received = false;

  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
    return channels_received;
  }

  if (protocol_state <= FRSKY_STATE_BIND_COMPLETE) {
    frsky_handle_bind();
    return channels_received;
  }

  if (frsky_d16_handle_packet()) {
    frsky_d16_set_rc_data();
    channels_received = true;
  }

  rx_lqi_update();

  if (profile.receiver.lqi_source == RX_LQI_SOURCE_PACKET_RATE) {
    rx_lqi_update_from_fps(LQI_FPS);
  }

  return channels_received;
}

#endif
