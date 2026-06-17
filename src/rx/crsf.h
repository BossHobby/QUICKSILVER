#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * CRSF protocol
 *
 * CRSF protocol uses a full-duplex uart connection between FC and receiver.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

#define CRSF_FRAME_SIZE_MAX 64
#define CRSF_PAYLOAD_SIZE_MAX 60
#define CRSF_MSP_PAYLOAD_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX - CRSF_FRAME_LENGTH_TYPE_CRC)
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_BAUDRATE_DEFAULT 420000

#define CRSF_DEVICEINFO_VERSION 0x01
#define CRSF_DEVICEINFO_PARAMETER_COUNT 0
#define CRSF_DEVICEINFO_HARDWARE_ID 0x51530000
#define CRSF_COMMAND_CRC_POLY 0xBA

enum {
  CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
  CRSF_FRAME_GPS_EXTENDED_PAYLOAD_SIZE = 20,
  CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
  CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
  CRSF_FRAME_LINK_STATISTICS_RX_PAYLOAD_SIZE = 5,
  CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE = 6,
  CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
  CRSF_FRAME_RC_CHANNELS_SUBSET_MIN_PAYLOAD_SIZE = 3,
  CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
  CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE = 16,
  CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
  CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
  CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
  CRSF_FRAME_LENGTH_ADDRESS = 1,     // length of ADDRESS field
  CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
  CRSF_FRAME_LENGTH_TYPE = 1,        // length of TYPE field
  CRSF_FRAME_LENGTH_CRC = 1,         // length of CRC field
  CRSF_FRAME_LENGTH_TYPE_CRC = 2,    // length of TYPE and CRC fields combined
  CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4 // length of Extended Dest/Origin, TYPE and CRC fields combined
};

typedef enum {
  CRSF_FRAMETYPE_GPS = 0x02,
  CRSF_FRAMETYPE_GPS_EXTENDED = 0x06,
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_RC_CHANNELS_SUBSET_PACKED = 0x17,
  CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C,
  CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0x1D,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_COMMAND = 0x32,
  CRSF_FRAMETYPE_MSP_REQ = 0x7A,  // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B, // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_t;

typedef enum {
  CRSF_COMMAND_SUBCMD_GENERAL = 0x0A,
  CRSF_COMMAND_SUBCMD_RX = 0x10,
} crsf_command_subcmd_t;

typedef enum {
  CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_PROPOSAL = 0x70,
  CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE = 0x71,
} crsf_command_general_subcmd_t;

typedef enum {
  CRSF_COMMAND_SUBCMD_RX_BIND = 0x01,
} crsf_command_rx_subcmd_t;

typedef enum {
  CRSF_ADDRESS_BROADCAST = 0x00,
  CRSF_ADDRESS_USB = 0x10,
  CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
  CRSF_ADDRESS_RESERVED1 = 0x8A,
  CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
  CRSF_ADDRESS_GPS = 0xC2,
  CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
  CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
  CRSF_ADDRESS_RESERVED2 = 0xCA,
  CRSF_ADDRESS_RACE_TAG = 0xCC,
  CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
  CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
  CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
} crsf_address_t;

typedef struct {
  uint8_t destination;
  uint8_t origin;
  uint8_t subcommand;
  uint8_t command;
} __attribute__((__packed__)) crsf_command_header_t;

typedef struct {
  crsf_command_header_t header;
  uint8_t port_id;
  uint32_t baudrate;
  uint8_t command_crc;
} __attribute__((__packed__)) crsf_speed_proposal_payload_t;

typedef struct {
  crsf_command_header_t header;
  uint8_t port_id;
  uint8_t response;
} __attribute__((__packed__)) crsf_speed_response_payload_t;

typedef struct {
  crsf_command_header_t header;
} __attribute__((__packed__)) crsf_bind_payload_t;

typedef struct {
  uint8_t address;
  uint8_t frame_length;
  uint8_t type;
} __attribute__((__packed__)) crsf_frame_header_t;

typedef struct {
  uint8_t destination;
  uint8_t origin;
} __attribute__((__packed__)) crsf_extended_header_t;

typedef struct {
  uint8_t address;
  uint8_t frame_length;
  uint8_t type;
  crsf_speed_response_payload_t payload;
  uint8_t command_crc;
  uint8_t crc;
} __attribute__((__packed__)) crsf_speed_response_frame_t;

typedef struct {
  uint8_t address;
  uint8_t frame_length;
  uint8_t type;
  crsf_bind_payload_t payload;
  uint8_t command_crc;
  uint8_t crc;
} __attribute__((__packed__)) crsf_bind_frame_t;

typedef enum {
  CRSF_TX_POWER_0_MW = 0,
  CRSF_TX_POWER_10_MW = 1,
  CRSF_TX_POWER_25_MW = 2,
  CRSF_TX_POWER_100_MW = 3,
  CRSF_TX_POWER_500_MW = 4,
  CRSF_TX_POWER_1000_MW = 5,
  CRSF_TX_POWER_2000_MW = 6,
  CRSF_TX_POWER_23_DBM_220_MW = 7,
  CRSF_TX_POWER_250_MW = CRSF_TX_POWER_23_DBM_220_MW,
  CRSF_TX_POWER_50_MW = 8,
  CRSF_TX_POWER_MAX = 9,
} crsf_tx_power_t;

typedef struct {
  uint16_t voltage;
  uint16_t current;
  uint32_t capacity_used : 24;
  uint32_t remaining : 8;
} __attribute__((__packed__)) crsf_battery_sensor_payload_t;

typedef struct {
  uint32_t latitude;
  uint32_t longitude;
  uint16_t ground_speed;
  uint16_t heading;
  uint16_t altitude;
  uint8_t satellites;
} __attribute__((__packed__)) crsf_gps_payload_t;

typedef struct {
  uint8_t fix_type;
  uint16_t n_speed;
  uint16_t e_speed;
  uint16_t v_speed;
  uint16_t h_speed_acc;
  uint16_t track_acc;
  uint16_t alt_ellipsoid;
  uint16_t h_acc;
  uint16_t v_acc;
  uint8_t reserved;
  uint8_t hdop;
  uint8_t vdop;
} __attribute__((__packed__)) crsf_gps_extended_payload_t;

typedef struct {
  char flight_mode[CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE];
} __attribute__((__packed__)) crsf_flight_mode_payload_t;

typedef struct {
  crsf_extended_header_t extended;
  uint8_t data[CRSF_MSP_PAYLOAD_SIZE_MAX];
} __attribute__((__packed__)) crsf_msp_response_payload_t;

typedef struct {
  crsf_frame_header_t header;
  crsf_battery_sensor_payload_t payload;
  uint8_t crc;
} __attribute__((__packed__)) crsf_battery_sensor_frame_t;

typedef struct {
  crsf_frame_header_t header;
  crsf_gps_payload_t payload;
  uint8_t crc;
} __attribute__((__packed__)) crsf_gps_frame_t;

typedef struct {
  crsf_frame_header_t header;
  crsf_gps_extended_payload_t payload;
  uint8_t crc;
} __attribute__((__packed__)) crsf_gps_extended_frame_t;

typedef struct {
  crsf_frame_header_t header;
  crsf_flight_mode_payload_t payload;
  uint8_t crc;
} __attribute__((__packed__)) crsf_flight_mode_frame_t;

typedef struct {
  crsf_frame_header_t header;
  crsf_msp_response_payload_t payload;
  uint8_t crc;
} __attribute__((__packed__)) crsf_msp_response_frame_t;

typedef struct {
  // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
  uint32_t chan0 : 11;
  uint32_t chan1 : 11;
  uint32_t chan2 : 11;
  uint32_t chan3 : 11;
  uint32_t chan4 : 11;
  uint32_t chan5 : 11;
  uint32_t chan6 : 11;
  uint32_t chan7 : 11;
  uint32_t chan8 : 11;
  uint32_t chan9 : 11;
  uint32_t chan10 : 11;
  uint32_t chan11 : 11;
  uint32_t chan12 : 11;
  uint32_t chan13 : 11;
  uint32_t chan14 : 11;
  uint32_t chan15 : 11;
} __attribute__((__packed__)) crsf_channels_t;

typedef struct {
  uint8_t config;
  uint8_t data[];
} __attribute__((__packed__)) crsf_channels_subset_payload_t;

typedef struct {
  uint8_t uplink_rssi_2;
  uint8_t uplink_rssi_1;
  uint8_t uplink_link_quality;
  int8_t uplink_snr;
  uint8_t active_antenna;
  uint8_t rf_mode;
  uint8_t uplink_tx_power;
  uint8_t downlink_rssi;
  uint8_t downlink_link_quality;
  int8_t downlink_snr;
} __attribute__((__packed__)) crsf_link_statistics_payload_t;

#if defined(__cplusplus)
static_assert(sizeof(crsf_link_statistics_payload_t) == CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE, "wrong crsf link statistics payload size");
#else
_Static_assert(sizeof(crsf_link_statistics_payload_t) == CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE, "wrong crsf link statistics payload size");
#endif

typedef struct {
  uint8_t uplink_rssi_2;
  uint8_t uplink_rssi_1;
  uint8_t uplink_link_quality;
  int8_t uplink_snr;
  uint8_t active_antenna;
  uint8_t rf_mode;
  uint8_t uplink_tx_power;
  uint8_t downlink_rssi;
  uint8_t downlink_link_quality;
  int8_t downlink_snr;
  uint16_t uplink_fps;
} crsf_stats_t;

typedef struct {
  uint8_t rssi_db;
  uint8_t rssi_percent;
  uint8_t link_quality;
  int8_t snr;
  uint8_t rf_power_db;
} __attribute__((__packed__)) crsf_link_stats_rx_t;

typedef struct {
  uint8_t rssi_db;
  uint8_t rssi_percent;
  uint8_t link_quality;
  int8_t snr;
  uint8_t rf_power_db;
  uint8_t fps;
} __attribute__((__packed__)) crsf_link_stats_tx_t;

typedef struct {
  uint8_t device_address;
  uint8_t frame_length;
  uint8_t type;
  uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsf_frame_def_t;

typedef union {
  uint8_t bytes[CRSF_FRAME_SIZE_MAX];
  crsf_frame_def_t frame;
} crsf_frame_t;

extern crsf_stats_t crsf_stats;

uint8_t crsf_crc8(uint8_t *data, uint16_t len);
uint8_t crsf_command_crc8(const uint8_t *data, uint8_t size);

uint32_t crsf_frame_speed_response(uint8_t *buf, uint8_t destination, uint8_t port_id, bool response);
uint32_t crsf_frame_bind(uint8_t *buf);
uint32_t crsf_tlm_frame_battery_sensor(uint8_t *buf);
uint32_t crsf_tlm_frame_gps(uint8_t *buf);
uint32_t crsf_tlm_frame_gps_extended(uint8_t *buf);
uint32_t crsf_tlm_frame_flight_mode(uint8_t *buf);
uint32_t crsf_tlm_frame_device_info(uint8_t *buf, uint8_t destination);
uint32_t crsf_tlm_frame_msp_resp(uint8_t *buf, uint8_t origin, const uint8_t *payload, uint8_t size);
