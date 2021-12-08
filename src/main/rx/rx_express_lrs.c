#include "rx_express_lrs.h"

#include <stdbool.h>
#include <string.h>

#include "control.h"
#include "drv_time.h"
#include "project.h"
#include "usb_configurator.h"

#if defined(RX_EXPRESS_LRS) && defined(USE_SX12XX)

#define SYNC_WORD 0x12 // default LoRa sync word

#define BUFFER_SIZE 8
#define DEVICE_ADDR (UID[5] & 0b111111)
#define CRC_CIPHER UID[4]

#define RC_DATA_PACKET 0b00
#define MSP_DATA_PACKET 0b01
#define TLM_PACKET 0b11
#define SYNC_PACKET 0b10

// Desired buffer time between Packet ISR and Tock ISR
#define PACKET_TO_TOCK_SLACK 200

#define FIFO_TX_BASE_ADDR_MAX 0b00000000
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
extern int32_t fhss_update_freq_correction(uint8_t value);
extern void fhss_randomize(int32_t seed);
extern uint32_t fhss_get_freq(uint16_t index);
extern uint32_t fhss_next_freq();
extern void fhss_reset();

extern void crc14_init();
extern uint16_t crc14_calc(uint8_t *data, uint8_t len, uint16_t crc);

extern void elrs_timer_init(uint32_t interval_us);
extern void elrs_timer_resume(uint32_t interval_us);
extern void elrs_timer_stop();

extern void elrs_phase_update(elrs_state_t state);
extern void elrs_phase_int_event(uint32_t time);
extern void elrs_phase_ext_event(uint32_t time);
extern void elrs_phase_reset();

float rx_rssi;

static volatile elrs_state_t elrs_state = DISCONNECTED;
static volatile bool already_hop = false;

// nonce that we THINK we are up to.
static uint8_t nonce_rx = 0;

static uint32_t current_rate = 0;

static uint8_t UID[6] = {221, 251, 226, 34, 222, 25};
// static uint8_t UID[6] = {0, 1, 2, 3, 4, 5};

static uint32_t sync_packet_time = 0;

static void elrs_set_frequency(int32_t FRQ) {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  uint8_t buf[3] = {
      (uint8_t)((FRQ >> 16) & 0xFF),
      (uint8_t)((FRQ >> 8) & 0xFF),
      (uint8_t)(FRQ & 0xFF),
  };
  sx12xx_write_reg_burst(SX127x_FRF_MSB, buf, 3);
}

static void elrs_set_rate(uint8_t index) {
  sx12xx_write_reg(SX127x_OP_MODE, SX127x_OPMODE_SLEEP);
  sx12xx_write_reg(SX127x_OP_MODE, SX127x_OPMODE_LORA);
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  sx12xx_write_reg(SX127x_IRQ_FLAGS, 0b11111111);

  sx12xx_write_reg(SX127x_PAYLOAD_LENGTH, BUFFER_SIZE);
  sx12xx_write_reg(SX127x_SYNC_WORD, SYNC_WORD);

  sx12xx_write_reg(SX127x_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR_MAX);
  sx12xx_write_reg(SX127x_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR_MAX);

  sx12xx_set_reg(SX127x_DIO_MAPPING_1, 0b11000000, 7, 6);

  sx12xx_write_reg(SX127x_LNA, SX127x_LNA_BOOST_ON);
  sx12xx_write_reg(SX127x_MODEM_CONFIG_3, SX1278_AGC_AUTO_ON | SX1278_LOW_DATA_RATE_OPT_OFF);
  sx12xx_set_reg(SX127x_OCP, SX127X_OCP_ON | SX127X_OCP_150MA, 5, 0);

  sx12xx_write_reg(SX127x_PREAMBLE_LSB, air_rate_config[index].preamble_len);

  sx12xx_set_reg(SX127x_INVERT_IQ, (uint8_t)(UID[5] & 0x01), 6, 6);

  sx12xx_set_mode(SX127x_OPMODE_STANDBY);
  sx12xx_write_reg(SX127x_PA_CONFIG, SX127x_PA_SELECT_BOOST | SX127x_MAX_OUTPUT_POWER | 0b0000);

  sx12xx_set_reg(SX127x_MODEM_CONFIG_2, air_rate_config[index].sf | SX127X_TX_MODE_SINGLE | SX1278_RX_CRC_MODE_OFF, 7, 2);
  if (air_rate_config[index].sf == SX127x_SF_6) {
    sx12xx_set_reg(SX127x_DETECT_OPTIMIZE, SX127x_DETECT_OPTIMIZE_SF_6, 2, 0);
    sx12xx_write_reg(SX127x_DETECTION_THRESHOLD, SX127x_DETECTION_THRESHOLD_SF_6);

    sx12xx_write_reg(SX127x_MODEM_CONFIG_1, air_rate_config[index].bw | air_rate_config[index].cr | SX1278_HEADER_IMPL_MODE);
  } else {
    sx12xx_set_reg(SX127x_DETECT_OPTIMIZE, SX127x_DETECT_OPTIMIZE_SF_7_12, 2, 0);
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

  current_rate = index;
}

static uint8_t packet[BUFFER_SIZE];

static void elrs_enter_rx() {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);
  sx12xx_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx12xx_set_mode(SX127x_OPMODE_RXCONTINUOUS);
}

static void elrs_enter_tx() {
  sx12xx_set_mode(SX127x_OPMODE_STANDBY);

  sx12xx_write_reg(SX127x_FIFO_ADDR_PTR, 0x0);
  sx12xx_write_fifo(packet, BUFFER_SIZE);

  sx12xx_set_mode(SX127x_OPMODE_TX);
}

static bool elrs_hop() {
  if (!already_hop) {
    return false;
  }

  const uint8_t modresult = (nonce_rx + 1) % air_rate_config[current_rate].fhss_hop_interval;
  if (modresult != 0) {
    return false;
  }

  elrs_set_frequency(fhss_next_freq());
  already_hop = true;

  const uint8_t tlm_mod = ((nonce_rx + 1) % air_rate_config[current_rate].tlm_interval);
  if (tlm_mod != 0 || air_rate_config[current_rate].tlm_interval == 0) {
    elrs_enter_rx();
  }
  return true;
}

static void elrs_tlm() {
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

static void elrs_read_packet() {
  sx12xx_read_fifo(packet, BUFFER_SIZE);
  sx12xx_write_reg(SX127x_IRQ_FLAGS, 0b11111111);
}

static bool elrs_vaild_packet() {
  if (!sx12xx_read_dio0()) {
    return false;
  }

  elrs_read_packet();

  const uint8_t type = packet[0] & 0b11;
  const uint16_t their_crc = (((uint16_t)(packet[0] & 0b11111100)) << 6) | packet[7];

  // reset first byte to type only so crc passes
  packet[0] = type;

  const uint16_t crc_initializer = (UID[4] << 8) | UID[5];
  const uint16_t our_crc = crc14_calc(packet, 7, crc_initializer);

  return their_crc == our_crc;
}

static void elrs_connection_lost() {
  elrs_state = DISCONNECTED;
  elrs_timer_stop();

  already_hop = false;

  fhss_reset();
  elrs_phase_reset();

  elrs_set_rate(current_rate);
  elrs_set_frequency(fhss_get_freq(0));
  elrs_enter_rx();
}

static void elrs_connection_tentative() {
  elrs_state = TENTATIVE;

  fhss_reset();
  elrs_phase_reset();

  elrs_timer_resume(air_rate_config[current_rate].interval);
}

// this is 180 out of phase with the other callback, occurs mid-packet reception
void elrs_handle_tick() {
  elrs_phase_update(elrs_state);
  nonce_rx++;
  already_hop = false;
}

void elrs_handle_tock() {
  elrs_phase_int_event(time_micros());

  const bool did_hop = elrs_hop();
  if (!did_hop) {
    const int32_t offset = fhss_update_freq_correction(((sx12xx_read_reg(SX127x_FEI_MSB) & 0b1000) >> 3) ? 1 : 0);
    sx12xx_write_reg(SX127x_PPMOFFSET, (uint8_t)offset);
  }
}

void elrs_process_packet(uint32_t packet_time) {
  elrs_phase_ext_event(packet_time + PACKET_TO_TOCK_SLACK);

  const uint8_t type = packet[0] & 0b11;
  switch (type) {
  case SYNC_PACKET: {
    if (packet[4] != UID[3] || packet[5] != UID[4] || packet[6] != UID[5]) {
      break;
    }

    sync_packet_time = time_millis();

    const uint8_t rate_index = ((packet[3] & 0b11000000) >> 6);
    const uint8_t telemetry_rate_index = ((packet[3] & 0b00111000) >> 3);
    //const uint8_t switch_mode = ((packet[3] & 0b00000110) >> 1);

    if (rate_index != current_rate) {
      current_rate = rate_index;
      elrs_connection_lost();
    }

    if (air_rate_config[current_rate].tlm_interval != tlm_ration_map[telemetry_rate_index]) {
      air_rate_config[current_rate].tlm_interval = tlm_ration_map[telemetry_rate_index];
    }

    if (elrs_state == DISCONNECTED ||
        (nonce_rx != packet[2]) ||
        (fhss_index != packet[1])) {
      fhss_index = packet[1];
      nonce_rx = packet[2];

      elrs_connection_tentative();
    }
    break;
  }

  default:
    break;
  }
}

void rx_init() {
  sx12xx_init();
  if (!sx12xx_detect()) {
    return;
  }

  const int32_t seed = ((int32_t)UID[2] << 24) + ((int32_t)UID[3] << 16) + ((int32_t)UID[4] << 8) + UID[5];
  fhss_randomize(seed);
  crc14_init();

  elrs_set_rate(2);
  elrs_timer_init(air_rate_config[current_rate].interval);
  elrs_set_frequency(fhss_get_freq(0));
  elrs_enter_rx();
}

void rx_check() {
  const uint32_t packet_time = time_micros();
  if (elrs_vaild_packet()) {
    elrs_process_packet(packet_time);
  }

  if (elrs_state == TENTATIVE && ((time_millis() - sync_packet_time) > rf_pref_params[current_rate].rf_mode_cycle_addtional_time)) {
    sync_packet_time = time_millis();
    elrs_connection_lost();
    return;
  }
}

#endif