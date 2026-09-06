#include "rx/crsf.h"

#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/target.h"
#include "io/gps.h"
#include "util/crc.h"
#include "util/util.h"

crsf_stats_t crsf_stats;

typedef struct {
  uint32_t serial_number;
  uint32_t hardware_id;
  uint32_t firmware_id;
  uint8_t parameter_count;
  uint8_t parameter_version;
} __attribute__((__packed__)) crsf_device_info_tail_t;

static uint32_t crsf_tlm_frame_finalize(uint8_t *buf, uint32_t payload_size) {
  crsf_frame_header_t *frame = (crsf_frame_header_t *)buf;
  frame->address = CRSF_SYNC_BYTE;
  frame->frame_length = payload_size + CRSF_FRAME_LENGTH_TYPE_CRC;
  buf[sizeof(*frame) + payload_size] = crc8_dvb_s2_data(0, &frame->type, payload_size + CRSF_FRAME_LENGTH_TYPE);
  return sizeof(*frame) + payload_size + CRSF_FRAME_LENGTH_CRC;
}

static uint32_t crsf_command_frame_finalize(uint8_t *buf, uint32_t payload_size) {
  crsf_frame_header_t *frame = (crsf_frame_header_t *)buf;
  const uint32_t command_crc_size = CRSF_FRAME_LENGTH_TYPE + payload_size;
  const uint32_t frame_crc_size = command_crc_size + CRSF_FRAME_LENGTH_CRC;
  uint8_t *command_crc = buf + sizeof(*frame) + payload_size;

  frame->address = CRSF_SYNC_BYTE;
  frame->frame_length = frame_crc_size + CRSF_FRAME_LENGTH_CRC;
  command_crc[0] = crsf_command_crc8(&frame->type, command_crc_size);
  command_crc[1] = crc8_dvb_s2_data(0, &frame->type, frame_crc_size);
  return sizeof(*frame) + payload_size + CRSF_FRAME_LENGTH_CRC + CRSF_FRAME_LENGTH_CRC;
}

uint8_t crsf_command_crc8(const uint8_t *data, uint8_t size) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < size; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (crc << 1) ^ CRSF_COMMAND_CRC_POLY : crc << 1;
    }
  }
  return crc;
}

static int16_t crsf_i16(int32_t value) {
  return (int16_t)constrain(value, (int32_t)INT16_MIN, (int32_t)INT16_MAX);
}

uint32_t crsf_frame_speed_response(uint8_t *buf, uint8_t destination, uint8_t port_id, bool response) {
  crsf_speed_response_frame_t *frame = (crsf_speed_response_frame_t *)buf;
  frame->type = CRSF_FRAMETYPE_COMMAND;
  frame->payload = (crsf_speed_response_payload_t){
      .header = {
          .destination = destination,
          .origin = CRSF_ADDRESS_FLIGHT_CONTROLLER,
          .subcommand = CRSF_COMMAND_SUBCMD_GENERAL,
          .command = CRSF_COMMAND_SUBCMD_GENERAL_CRSF_SPEED_RESPONSE,
      },
      .port_id = port_id,
      .response = response,
  };
  return crsf_command_frame_finalize(buf, sizeof(frame->payload));
}

uint32_t crsf_frame_bind(uint8_t *buf) {
  crsf_bind_frame_t *frame = (crsf_bind_frame_t *)buf;
  frame->type = CRSF_FRAMETYPE_COMMAND;
  frame->payload = (crsf_bind_payload_t){
      .header = {
          .destination = CRSF_ADDRESS_CRSF_RECEIVER,
          .origin = CRSF_ADDRESS_FLIGHT_CONTROLLER,
          .subcommand = CRSF_COMMAND_SUBCMD_RX,
          .command = CRSF_COMMAND_SUBCMD_RX_BIND,
      },
  };
  return crsf_command_frame_finalize(buf, sizeof(frame->payload));
}

/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name ( Null terminated string )
uint32_t    Serial Number
uint32_t    Hardware ID
uint32_t    Firmware ID
uint8_t     Parameters Count
uint8_t     0x01 (Parameter version 1)
*/
uint32_t crsf_tlm_frame_device_info(uint8_t *buf, uint8_t destination) {
  crsf_frame_header_t *frame = (crsf_frame_header_t *)buf;
  frame->type = CRSF_FRAMETYPE_DEVICE_INFO;

  crsf_extended_header_t *extended = (crsf_extended_header_t *)(buf + sizeof(*frame));
  extended->destination = destination;
  extended->origin = CRSF_ADDRESS_FLIGHT_CONTROLLER;

  uint8_t *name = (uint8_t *)(extended + 1);
  uint8_t name_size = 0;
  while (name_size < sizeof(profile.meta.name) && profile.meta.name[name_size])
    name_size++;

  memcpy(name, profile.meta.name, name_size);
  name[name_size++] = 0;

  crsf_device_info_tail_t *tail = (crsf_device_info_tail_t *)(name + name_size);
  *tail = (crsf_device_info_tail_t){
      .serial_number = __builtin_bswap32(get_chip_uid()),
      .hardware_id = __builtin_bswap32(CRSF_DEVICEINFO_HARDWARE_ID),
      .firmware_id = __builtin_bswap32(target_info.firmware_version),
      .parameter_count = CRSF_DEVICEINFO_PARAMETER_COUNT,
      .parameter_version = CRSF_DEVICEINFO_VERSION,
  };

  return crsf_tlm_frame_finalize(buf, CRSF_FRAME_ORIGIN_DEST_SIZE + name_size + sizeof(*tail));
}

// Telemetry sending back to receiver (only voltage for now)
/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/

/*
0x08 Battery sensor (CRSF_FRAMETYPE_BATTERY_SENSOR)
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Fuel ( drawn mAh )
uint8_t     Battery remaining ( percent )
*/
uint32_t crsf_tlm_frame_battery_sensor(uint8_t *buf) {
  crsf_battery_sensor_frame_t *frame = (crsf_battery_sensor_frame_t *)buf;
  frame->header.type = CRSF_FRAMETYPE_BATTERY_SENSOR;

  frame->payload.voltage = __builtin_bswap16((uint16_t)(state.vbat_cell_avg * 10));
  frame->payload.current = __builtin_bswap16((uint16_t)(state.ibat_filtered * 10));
  frame->payload.capacity_used = __builtin_bswap32((uint32_t)state.ibat_drawn) >> 8;
  frame->payload.remaining = 100;
  return crsf_tlm_frame_finalize(buf, CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE);
}

uint32_t crsf_tlm_frame_gps(uint8_t *buf) {
  crsf_gps_frame_t *frame = (crsf_gps_frame_t *)buf;
  frame->header.type = CRSF_FRAMETYPE_GPS;

  frame->payload.latitude = __builtin_bswap32((uint32_t)state.gps_coord.lat);
  frame->payload.longitude = __builtin_bswap32((uint32_t)state.gps_coord.lon);

  frame->payload.ground_speed = __builtin_bswap16((uint16_t)(state.gps_speed * 3.6f * 100));
  frame->payload.heading = __builtin_bswap16((uint16_t)(state.gps_heading * 100));
  frame->payload.altitude = __builtin_bswap16((uint16_t)(state.gps_altitude + 1000));
  frame->payload.satellites = state.gps_sats;

  return crsf_tlm_frame_finalize(buf, CRSF_FRAME_GPS_PAYLOAD_SIZE);
}

uint32_t crsf_tlm_frame_gps_extended(uint8_t *buf) {
  crsf_gps_extended_frame_t *frame = (crsf_gps_extended_frame_t *)buf;
  frame->header.type = CRSF_FRAMETYPE_GPS_EXTENDED;

  frame->payload.fix_type = gps_status.fix_type;

  frame->payload.n_speed = __builtin_bswap16((uint16_t)crsf_i16(gps_status.vel_n / 10));
  frame->payload.e_speed = __builtin_bswap16((uint16_t)crsf_i16(gps_status.vel_e / 10));
  frame->payload.v_speed = __builtin_bswap16((uint16_t)crsf_i16(-(gps_status.vel_d / 10)));
  frame->payload.h_speed_acc = __builtin_bswap16((uint16_t)MIN(gps_status.speed_acc / 10, (uint32_t)INT16_MAX));
  frame->payload.track_acc = __builtin_bswap16((uint16_t)MIN(gps_status.head_acc / 10000, (uint32_t)INT16_MAX));
  frame->payload.alt_ellipsoid = __builtin_bswap16((uint16_t)crsf_i16(gps_status.height / 1000));
  frame->payload.h_acc = __builtin_bswap16((uint16_t)MIN(gps_status.h_acc / 10, (uint32_t)INT16_MAX));
  frame->payload.v_acc = __builtin_bswap16((uint16_t)MIN(gps_status.v_acc / 10, (uint32_t)INT16_MAX));
  frame->payload.reserved = 0;
  frame->payload.hdop = (uint8_t)MIN(gps_status.pdop / 10, (uint16_t)UINT8_MAX);
  frame->payload.vdop = frame->payload.hdop;

  return crsf_tlm_frame_finalize(buf, CRSF_FRAME_GPS_EXTENDED_PAYLOAD_SIZE);
}

uint32_t crsf_tlm_frame_flight_mode(uint8_t *buf) {
  crsf_flight_mode_frame_t *frame = (crsf_flight_mode_frame_t *)buf;
  frame->header.type = CRSF_FRAMETYPE_FLIGHT_MODE;
  memset(&frame->payload, 0, sizeof(frame->payload));

  const char *flight_mode = control_flight_mode_name();
  uint8_t size = 0;
  while (size < sizeof(frame->payload.flight_mode) - 2 && flight_mode[size]) {
    frame->payload.flight_mode[size] = flight_mode[size];
    size++;
  }

  if (!flags.arm_state && size < sizeof(frame->payload.flight_mode) - 1)
    frame->payload.flight_mode[size++] = flags.arming_disabled_flags == ARMING_DISABLED_NONE ? '*' : '!';

  return crsf_tlm_frame_finalize(buf, sizeof(frame->payload));
}

uint32_t crsf_tlm_frame_msp_resp(uint8_t *buf, uint8_t origin, const uint8_t *payload, uint8_t size) {
  if (size > CRSF_MSP_PAYLOAD_SIZE_MAX)
    size = CRSF_MSP_PAYLOAD_SIZE_MAX;

  crsf_msp_response_frame_t *frame = (crsf_msp_response_frame_t *)buf;
  frame->header.type = CRSF_FRAMETYPE_MSP_RESP;
  frame->payload.extended.destination = origin;
  frame->payload.extended.origin = CRSF_ADDRESS_FLIGHT_CONTROLLER;
  memcpy(frame->payload.data, payload, size);
  return crsf_tlm_frame_finalize(buf, size + CRSF_FRAME_ORIGIN_DEST_SIZE);
}
