#pragma once

#include <stdint.h>

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
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
#define CRSF_MSP_PAYLOAD_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX - CRSF_FRAME_LENGTH_EXT_TYPE_CRC)
#define CRSF_SYNC_BYTE 0xC8

#define CRSF_DEVICEINFO_VERSION 0x01
#define CRSF_DEVICEINFO_PARAMETER_COUNT 0

enum {
  CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
  CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
  CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
  CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
  CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
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
  CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
  CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
  CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
  CRSF_FRAMETYPE_ATTITUDE = 0x1E,
  CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
  CRSF_FRAMETYPE_DEVICE_PING = 0x28,
  CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
  CRSF_FRAMETYPE_MSP_REQ = 0x7A,  // response request using msp sequence as command
  CRSF_FRAMETYPE_MSP_RESP = 0x7B, // reply with 58 byte chunked binary
  CRSF_FRAMETYPE_MSP_WRITE = 0x7C // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
} crsf_frame_type_t;

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
} crsf_stats_t;

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

uint8_t crsf_crc8(uint8_t *data, uint16_t len);

void crsf_tlm_frame_start(uint8_t *buf);
uint32_t crsf_tlm_frame_battery_sensor(uint8_t *buf);
uint32_t crsf_tlm_frame_device_info(uint8_t *buf);
uint32_t crsf_tlm_frame_finish(uint8_t *buf, uint32_t payload_size);
uint32_t crsf_tlm_frame_msp_resp(uint8_t *buf, uint8_t origin, uint8_t *payload, uint8_t size);