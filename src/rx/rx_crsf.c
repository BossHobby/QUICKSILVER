#include "rx_crsf.h"

#include "flight/control.h"
#include "profile.h"

uint8_t crsf_crc8(uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc = crc ^ data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc;
}

void crsf_tlm_frame_start(uint8_t *buf) {
  buf[0] = CRSF_SYNC_BYTE;
}

uint32_t crsf_tlm_frame_finish(uint8_t *buf, uint32_t payload_size) {
  buf[payload_size + CRSF_FRAME_LENGTH_TYPE_CRC + 1] = crsf_crc8(buf + 2, payload_size + 1);
  return payload_size + CRSF_FRAME_LENGTH_TYPE_CRC + 1;
}

/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name ( Null terminated string )
uint32_t    Null Bytes
uint32_t    Null Bytes
uint32_t    Null Bytes
uint8_t     255 (Max MSP Parameter)
uint8_t     0x01 (Parameter version 1)
*/
uint32_t crsf_tlm_frame_device_info(uint8_t *buf) {
  buf[2] = CRSF_FRAMETYPE_DEVICE_INFO;
  buf[3] = CRSF_ADDRESS_RADIO_TRANSMITTER;
  buf[4] = CRSF_ADDRESS_FLIGHT_CONTROLLER;

  uint32_t offset = 5;
  for (uint32_t i = 0; i < 36; i++) {
    buf[offset++] = profile.meta.name[i];
  }
  for (uint32_t i = 0; i < 12; i++) {
    buf[offset++] = 0x0;
  }
  buf[offset++] = CRSF_DEVICEINFO_PARAMETER_COUNT;
  buf[offset++] = CRSF_DEVICEINFO_VERSION;

  // set frame length
  buf[1] = offset;

  return offset - 2;
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
  buf[1] = CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
  buf[2] = CRSF_FRAMETYPE_BATTERY_SENSOR;
  buf[3] = (int)(state.vbat_filtered * 10) >> 8;
  buf[4] = (int)(state.vbat_filtered * 10);
  buf[5] = (int)(state.ibat_filtered * 10) >> 8;
  buf[6] = (int)(state.ibat_filtered * 10);
  const uint32_t mah_drawn = 0;
  const uint8_t battery_remaining_percentage = 0;
  buf[7] = mah_drawn >> 16;
  buf[8] = mah_drawn >> 8;
  buf[9] = mah_drawn;
  buf[10] = battery_remaining_percentage;
  return CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE;
}
