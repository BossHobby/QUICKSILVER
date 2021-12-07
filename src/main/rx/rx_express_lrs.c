#include "rx_express_lrs.h"

#include <string.h>

#include "control.h"
#include "drv_time.h"
#include "project.h"
#include "usb_configurator.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX12XX)

#define BUFFER_SIZE 8
#define SYNC_WORD UID[3]
#define DEVICE_ADDR (UID[5] & 0b111111)
#define CRC_CIPHER UID[4]

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define TLM_PACKET 0b11
#define SYNC_PACKET 0b10

//  7     0     allocate the entire FIFO buffer for TX only
#define FIFO_TX_BASE_ADDR_MAX 0b00000000

//  7     0     allocate the entire FIFO buffer for RX only
#define FIFO_RX_BASE_ADDR_MAX 0b00000000

static expresslrs_mod_settings_t air_rate_config[4] = {
    {0, RATE_200HZ, SX127x_BW_500_00_KHZ, SX127x_SF_6, SX127x_CR_4_7, 5000, TLM_RATIO_1_64, 4, 8},
    {1, RATE_100HZ, SX127x_BW_500_00_KHZ, SX127x_SF_7, SX127x_CR_4_7, 10000, TLM_RATIO_1_64, 4, 8},
    {2, RATE_50HZ, SX127x_BW_500_00_KHZ, SX127x_SF_8, SX127x_CR_4_7, 20000, TLM_RATIO_NO_TLM, 4, 10},
    {3, RATE_25HZ, SX127x_BW_500_00_KHZ, SX127x_SF_9, SX127x_CR_4_7, 40000, TLM_RATIO_NO_TLM, 4, 10},
};

static const expresslrs_rf_pref_params_t rf_pref_params[4] = {
    {0, RATE_200HZ, -112, 4380, 3000, 2500, 2000, 4000},
    {1, RATE_100HZ, -117, 8770, 3500, 2500, 2000, 4000},
    {2, RATE_50HZ, -120, 17540, 4000, 2500, 2000, 4000},
    {3, RATE_25HZ, -123, 17540, 6000, 4000, 2000, 4000},
};

extern uint8_t fhss_index;
extern void fhss_update_freq_correction(uint8_t value);
extern void fhss_randomize(int32_t seed);
extern uint32_t fhss_get_freq(uint16_t index);
extern uint32_t fhss_next_freq();

extern void crc14_init();
extern uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc);

float rx_rssi;

// nonce that we THINK we are up to.
static uint8_t nonce_rx = 0;

static uint32_t current_rate = 0;
static uint32_t max_sync_delay = 0;

static uint8_t UID[6] = {'h', 'a', 'n', 'f', 'e', 'r'};

void elrs_set_frequency(int32_t FRQ) {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  uint8_t buf[4] = {
      (uint8_t)((FRQ >> 16) & 0xFF),
      (uint8_t)((FRQ >> 8) & 0xFF),
      (uint8_t)(FRQ & 0xFF),
      0, // 32-bit aligned
  };
  sx12xx_write_reg_burst(SX127x_FRF_MSB, buf, 4);
}

void elrs_set_rate(uint8_t index) {
  sx12xx_write_reg(SX127x_OP_MODE, 0x8 | SX127x_OPMODE_SLEEP);
  sx12xx_write_reg(SX127x_OP_MODE, 0x8 | SX127x_OPMODE_LORA);
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  sx12xx_write_reg(SX127x_PAYLOAD_LENGTH, BUFFER_SIZE);
  sx12xx_write_reg(SX127x_SYNC_WORD, SYNC_WORD);

  sx12xx_write_reg(SX127x_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR_MAX);
  sx12xx_write_reg(SX127x_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR_MAX);

  sx12xx_write_reg(SX127x_DIO_MAPPING_1, 0b11000000);

  sx12xx_write_reg(SX127x_LNA, SX127x_LNA_BOOST_ON);
  sx12xx_write_reg(SX127x_MODEM_CONFIG_3, SX1278_AGC_AUTO_ON | SX1278_LOW_DATA_RATE_OPT_OFF);
  sx12xx_write_reg(SX127x_OCP, SX127X_OCP_ON | SX127X_OCP_150MA);

  sx12xx_write_reg(SX127x_PREAMBLE_LSB, air_rate_config[index].preamble_len);

  sx12xx_set_mode(SX127x_OPMODE_STANDBY);
  sx12xx_write_reg(SX127x_PA_CONFIG, SX127x_PA_SELECT_BOOST | SX127x_MAX_OUTPUT_POWER | 0b1111);

  sx12xx_write_reg(SX127x_MODEM_CONFIG_2, air_rate_config[index].sf | SX127X_TX_MODE_SINGLE | SX1278_RX_CRC_MODE_OFF);
  if (air_rate_config[index].sf == SX127x_SF_6) {
    sx12xx_write_reg(SX127x_DETECT_OPTIMIZE, SX127x_DETECT_OPTIMIZE_SF_6);
    sx12xx_write_reg(SX127x_DETECTION_THRESHOLD, SX127x_DETECTION_THRESHOLD_SF_6);

    sx12xx_write_reg(SX127x_MODEM_CONFIG_1, air_rate_config[index].bw | air_rate_config[index].cr | SX1278_HEADER_IMPL_MODE);
  } else {
    sx12xx_write_reg(SX127x_DETECT_OPTIMIZE, 0b10000000 | SX127x_DETECT_OPTIMIZE_SF_7_12);
    sx12xx_write_reg(SX127x_DETECTION_THRESHOLD, SX127x_DETECTION_THRESHOLD_SF_7_12);

    sx12xx_write_reg(SX127x_MODEM_CONFIG_1, air_rate_config[index].bw | air_rate_config[index].cr | SX1278_HEADER_IMPL_MODE);
  }

  if (air_rate_config[index].bw == SX127x_BW_500_00_KHZ) {
    //datasheet errata reconmendation http://caxapa.ru/thumbs/972894/SX1276_77_8_ErrataNote_1.1_STD.pdf
    sx12xx_write_reg(0x36, 0x02);
    sx12xx_write_reg(0x3a, 0x64);
  } else {
    sx12xx_write_reg(0x36, 0x03);
  }

  max_sync_delay = air_rate_config[index].interval;
  current_rate = index;
}

static uint8_t packet[BUFFER_SIZE];

void elrs_enter_rx() {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);
  sx12xx_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx12xx_set_mode(SX127x_OPMODE_RXCONTINUOUS);
}

void elrs_enter_tx() {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  sx12xx_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx12xx_write_fifo(packet, BUFFER_SIZE);

  sx12xx_set_mode(SX127x_OPMODE_TX);
}

void elrs_hop() {
  const uint8_t modresult = (nonce_rx + 1) % air_rate_config[current_rate].fhss_hop_interval;
  if (modresult != 0) {
    return;
  }

  elrs_set_frequency(fhss_next_freq());

  if (air_rate_config[current_rate].tlm_interval == 0) {
    elrs_enter_rx();
  } else if (((nonce_rx + 1) % air_rate_config[current_rate].tlm_interval) != 0) {
    elrs_enter_rx();
  }
}

void elrs_tlm() {
  if (air_rate_config[current_rate].tlm_interval == 0) {
    return;
  }

  const uint8_t modresult = (nonce_rx + 1) % air_rate_config[current_rate].tlm_interval;
  if (modresult != 0) {
    return;
  }

  packet[0] = (DEVICE_ADDR << 2) + 0b11;
  packet[1] = 0x14;
  packet[2] = 88;
  packet[3] = 0;
  packet[4] = 0;
  packet[5] = 0;
  packet[6] = 0;
  packet[7] = 0; // TODO: CRC
}

void elrs_read_packet() {
  uint8_t addr = sx12xx_read_reg(SX127x_FIFO_RX_CURRENT_ADDR);
  sx12xx_write_reg(SX127x_FIFO_ADDR_PTR, addr);

  sx12xx_read_fifo(packet, BUFFER_SIZE);

  sx12xx_write_reg(SX127x_IRQ_FLAGS, 0b11111111);
}

uint8_t elrs_vaild_packet() {
  elrs_read_packet();
  fhss_update_freq_correction(((sx12xx_read_reg(SX127x_FEI_MSB) & 0b1000) >> 3) ? 1 : 0);

  const uint8_t type = packet[0] & 0b11;
  const uint16_t their_crc = (((uint16_t)(packet[0] & 0b11111100)) << 6) | packet[7];

  // reset first byte to type only so crc passes
  packet[0] = type;

  const uint16_t crc_initializer = (UID[4] << 8) | UID[5];
  const uint16_t our_crc = crc14_calc(packet, 7, crc_initializer);
  if (their_crc != our_crc) {
    return 0;
  }

  const uint8_t addr = (packet[0] & 0b11111100) >> 2;
  if (addr != DEVICE_ADDR) {
    return 0;
  }
  return 1;
}

uint8_t elrs_process_packet() {

  const uint8_t type = packet[0] & 0b11;
  switch (type) {
  case RC_DATA_PACKET: {
    state.rx.axis[0] = (packet[1] << 3) + ((packet[5] & 0b11100000) >> 5);
    state.rx.axis[1] = (packet[2] << 3) + ((packet[5] & 0b00011100) >> 2);
    state.rx.axis[2] = (packet[3] << 3) + ((packet[5] & 0b00000011) << 1) + (packet[6] & 0b10000000 >> 7);
    state.rx.axis[3] = (packet[4] << 3) + ((packet[6] & 0b01110000) >> 4);
    state.aux[AUX_CHANNEL_0] = (packet[6] & 0b00001000) ? 1 : 0;
    state.aux[AUX_CHANNEL_1] = (packet[6] & 0b00000100) ? 1 : 0;
    state.aux[AUX_CHANNEL_2] = (packet[6] & 0b00000010) ? 1 : 0;
    state.aux[AUX_CHANNEL_3] = (packet[6] & 0b00000001) ? 1 : 0;
    break;
  }

  case SYNC_PACKET: {
    if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
      break;
    }
    if ((nonce_rx == packet[2]) && (fhss_index == packet[1])) {
      //GotConnection();
    } else {
      fhss_index = packet[1];
      nonce_rx = packet[2];
    }

    break;
  }

  default:
    break;
  }

  return 1;
}

void rx_init() {
  sx12xx_init();
  if (!sx12xx_detect()) {
    return;
  }

  fhss_randomize(((int64_t)UID[2] << 24) + ((int64_t)UID[3] << 16) + ((int64_t)UID[4] << 8) + UID[5]);
  crc14_init();

  elrs_set_rate(0);
  elrs_set_frequency(fhss_get_freq(0));
  elrs_enter_rx();
}

typedef enum {
  CONNECTION_LOST,
  DISCONNECTED,
  TENTATIVE,
  CONNECTED
} elrs_state_t;

void rx_check() {
  static elrs_state_t state = DISCONNECTED;

  static uint32_t packet_time = 0;
  static uint32_t sync_packet_time = 0;

  static uint8_t timer_tick = 0;

  static uint8_t did_hop = 0;
  static uint8_t did_tlm = 0;

  switch (state) {
  case CONNECTION_LOST:
    elrs_set_frequency(fhss_get_freq(0));
    elrs_enter_rx();
    state = DISCONNECTED;
    break;

  case DISCONNECTED: {
    if (sx12xx_read_dio0() && elrs_vaild_packet()) {
      packet_time = timer_micros();

      const uint8_t type = packet[0] & 0b11;
      if (type != SYNC_PACKET) {
        break;
      }

      if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
        break;
      }

      sync_packet_time = timer_millis();

      quic_debugf("ELRS: sync packet fhss %d vs %d nonce %d vs %d (disconnected)", packet[1], fhss_index, packet[2], nonce_rx);

      const uint8_t rate_index = ((packet[3] & 0b11100000) >> 5);
      if (rate_index != current_rate) {
        current_rate = rate_index;
      }
      const uint8_t telemetry_rate_index = ((packet[3] & 0b00011100) >> 2);
      if (air_rate_config[current_rate].tlm_interval != tlm_ration_map[telemetry_rate_index]) {
        air_rate_config[current_rate].tlm_interval = tlm_ration_map[telemetry_rate_index];
      }

      if ((nonce_rx == packet[2]) && (fhss_index == packet[1])) {
        //state = CONNECTED;
      } else {
        fhss_index = packet[1];
        nonce_rx = packet[2];

        state = TENTATIVE;
      }
    }
    break;
  }

  case TENTATIVE: {
    if (!did_hop && (timer_micros() - packet_time) > ((max_sync_delay / 2) - 200)) {
      nonce_rx++;

      elrs_hop();
      //elrs_tlm();

      did_hop = 1;
      break;
    }

    if ((timer_micros() - packet_time) > (max_sync_delay + 50)) {
      packet_time = timer_micros();
      //sx12xx_reset_dio();
      did_hop = 0;
      break;
    }

    if (did_hop && sx12xx_read_dio0() && elrs_vaild_packet()) {
      packet_time = timer_micros();
      did_hop = 0;

      const uint8_t type = packet[0] & 0b11;
      switch (type) {
      case SYNC_PACKET:
        if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
          break;
        }

        sync_packet_time = timer_millis();
        //quic_debugf("ELRS: sync packet fhss %d vs %d nonce %d vs %d (tentative)", packet[1], fhss_index, packet[2], nonce_rx);

        const uint8_t rate_index = ((packet[3] & 0b11100000) >> 5);
        if (rate_index != current_rate) {
          current_rate = rate_index;
        }

        const uint8_t telemetry_rate_index = ((packet[3] & 0b00011100) >> 2);
        if (air_rate_config[current_rate].tlm_interval != tlm_ration_map[telemetry_rate_index]) {
          air_rate_config[current_rate].tlm_interval = tlm_ration_map[telemetry_rate_index];
        }

        if ((nonce_rx == packet[2]) && (fhss_index == packet[1])) {
          quic_debugf("ELRS: CONNECTED");
          //state = CONNECTED;
        } else {
          fhss_index = packet[1];
          nonce_rx = packet[2];
          state = TENTATIVE;
        }
        break;

      default:
        break;
      }
      break;
    }

    if ((timer_millis() - sync_packet_time) > (rf_pref_params[current_rate].rf_mode_cycle_addtional_time + 5000)) {
      sync_packet_time = timer_millis();

      state = CONNECTION_LOST;
      break;
    }

    break;
  }

  default:
    break;
  }
}

#endif