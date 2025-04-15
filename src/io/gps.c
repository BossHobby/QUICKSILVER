#include "gps.h"

#include <stdlib.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "flight/control.h"
#include "util/ring_buffer.h"

#ifdef USE_GPS

#define UBX_BAUDRATE 115200

#define UBX_SYNC1 0xb5
#define UBX_SYNC2 0x62

#define UBX_MON_VER 0x0a04
#define UBX_NAV_PVT 0x0107
#define UBX_CFG_PRT 0x0600
#define UBX_ACK_ACK 0x0501
#define UBX_ACK_NAK 0x0500

typedef enum {
  SYNC1,
  SYNC2,
  CLASS,
  ID,
  LENGTH,
  PAYLOAD,
  CHECKSUM,
} gps_parser_state_t;

typedef enum {
  VER_INVALID,
  VER_M5 = 0x00040005,
  VER_M6 = 0x00040007,
  VER_M7 = 0x00070000,
  VER_M8 = 0x00080000,
  VER_M9 = 0x00190000,
  VER_M10 = 0x000A0000,
} gps_version_t;

typedef struct {
  uint8_t portID;
  uint8_t reserved1;
  uint16_t txReady;
  uint32_t mode;
  uint32_t baudRate;
  uint16_t inProtoMask;
  uint16_t outProtoMask;
  uint16_t flags;
  uint16_t reserved2;
} ubx_cfg_prt_t;

/* Valid field bits */
#define VALID_DATE 0x01           // Valid UTC Date
#define VALID_TIME 0x02           // Valid UTC Time of Day
#define VALID_FULLY_RESOLVED 0x04 // UTC Time of Day has been fully resolved
#define VALID_MAG 0x08            // Valid Magnetic Declination

/* Flags field bits */
#define FLAGS_GNSS_FIX_OK 0x01    // Valid fix (i.e. within DOP & accuracy masks)
#define FLAGS_DIFF_SOLN 0x02      // Differential corrections were applied
#define FLAGS_PSM_STATE_MASK 0x1C // Power Save Mode state (3 bits)
#define FLAGS_PSM_STATE_SHIFT 2   // PSM state bit shift
#define FLAGS_HEAD_VEH_VALID 0x20 // heading of vehicle is valid
#define FLAGS_CARR_SOLN_MASK 0xC0 // Carrier phase range solution status (2 bits)
#define FLAGS_CARR_SOLN_SHIFT 6   // Carrier solution bit shift
// Carrier phase solution status:
// 0: no carrier phase solution
// 1: float solution (no fixed integer carrier phase)
// 2: fixed solution (integer carrier phase)

/* Flags2 field bits */
#define FLAGS2_CONFIRMED_AVAI 0x01 // Information about UTC Date and Time of Day validity confirmation is available
#define FLAGS2_CONFIRMED_DATE 0x02 // UTC Date validity could be confirmed
#define FLAGS2_CONFIRMED_TIME 0x04 // UTC Time of Day validity could be confirmed
#define FLAGS2_DR_SOLUTION_OK 0x08 // If the odometer dead-reckoning solution is valid

/* Flags3 field bits */
#define FLAGS3_INVALID_LLAT 0x01 // Invalid lon/lat
#define FLAGS3_INVALID_ALT 0x02  // Invalid altitude
#define FLAGS3_INVALID_VEL 0x04  // Invalid velocity
#define FLAGS3_INVALID_MAG 0x08  // Invalid magnetic declination
#define FLAGS3_INVALID_POS 0x10  // Position invalid
#define FLAGS3_INVALID_DAGE 0x20 // Differential age exceeded age limit - non-RTK
#define FLAGS3_INVALID_BASE 0x40 // Invalid baseline - RTK
#define FLAGS3_INVALID_HEAD 0x80 // Invalid heading

typedef struct {
  uint32_t iTOW;        // GPS time of week [ms]
  uint16_t year;        // Year (UTC)
  uint8_t month;        // Month, range 1..12 (UTC)
  uint8_t day;          // Day of month, range 1..31 (UTC)
  uint8_t hour;         // Hour of day, range 0..23 (UTC)
  uint8_t min;          // Minute of hour, range 0..59 (UTC)
  uint8_t sec;          // Seconds of minute, range 0..60 (UTC)
  uint8_t valid;        // Validity flags
  uint32_t tAcc;        // Time accuracy estimate [ns]
  int32_t nano;         // Fraction of second, range -1e9..1e9 (UTC) [ns]
  uint8_t fixType;      // GNSSfix Type: 0=no fix, 1=DR only, 2=2D, 3=3D, 4=GNSS+DR, 5=Time only
  uint8_t flags;        // Fix status flags
  uint8_t flags2;       // Additional flags
  uint8_t numSV;        // Number of satellites used in Nav Solution
  int32_t lon;          // Longitude [1e-7 deg]
  int32_t lat;          // Latitude [1e-7 deg]
  int32_t height;       // Height above ellipsoid [mm]
  int32_t hMSL;         // Height above mean sea level [mm]
  uint32_t hAcc;        // Horizontal accuracy estimate [mm]
  uint32_t vAcc;        // Vertical accuracy estimate [mm]
  int32_t velN;         // NED north velocity [mm/s]
  int32_t velE;         // NED east velocity [mm/s]
  int32_t velD;         // NED down velocity [mm/s]
  int32_t gSpeed;       // Ground Speed (2-D) [mm/s]
  int32_t headMot;      // Heading of motion (2-D) [1e-5 deg]
  uint32_t sAcc;        // Speed accuracy estimate [mm/s]
  uint32_t headAcc;     // Heading accuracy estimate [1e-5 deg]
  uint16_t pDOP;        // Position DOP [0.01]
  uint16_t flags3;      // Additional flags
  uint8_t reserved1[4]; // Reserved
  int32_t headVeh;      // Heading of vehicle (2-D), only valid when headVehValid is set [1e-5 deg]
  int16_t magDec;       // Magnetic declination [1e-2 deg]
  uint16_t magAcc;      // Magnetic declination accuracy [1e-2 deg]
} ubx_nav_pvt_t;

static uint8_t tx_data[128];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = 128,
};

static uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_gps = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

static gps_version_t version = VER_INVALID;

static void gps_crc(uint8_t *ck_a, uint8_t *ck_b, uint8_t data) {
  *ck_a = *ck_a + data;
  *ck_b = *ck_b + *ck_a;
}

static bool gps_send_message(const uint16_t class_id, const uint8_t *payload, const uint16_t size) {
  if (serial_bytes_free(&serial_gps) < 5 + 2 + size)
    return false;

  const uint8_t header[] = {
      UBX_SYNC1,
      UBX_SYNC2,
      class_id >> 8,
      class_id & 0xff,
      size & 0xFF,
      size >> 8,
  };

  uint8_t crc[2] = {0, 0};
  for (uint32_t i = 0; i < sizeof(header) - 2; i++) {
    gps_crc(&crc[0], &crc[1], header[2 + i]);
  }
  for (uint32_t i = 0; i < size; i++) {
    gps_crc(&crc[0], &crc[1], payload[i]);
  }

  serial_write_bytes(&serial_gps, header, sizeof(header));
  serial_write_bytes(&serial_gps, payload, size);
  serial_write_bytes(&serial_gps, crc, sizeof(crc));

  return true;
}

static void gps_handle_packet(const uint16_t class_id, const uint8_t *payload, const uint16_t size) {
  switch (class_id) {
  case UBX_NAV_PVT: {
    ubx_nav_pvt_t *nav_pvt = (ubx_nav_pvt_t *)payload;
    state.gps_sats = nav_pvt->numSV | (nav_pvt->flags & FLAGS_GNSS_FIX_OK ? 0x80 : 0x0);
    state.gps_lon = nav_pvt->lon;
    state.gps_lat = nav_pvt->lat;
    state.gps_altitude = nav_pvt->hMSL;
    break;
  }
  case UBX_MON_VER: {
    version = strtoul((const char *)(payload + 30), NULL, 16);
    if (version != VER_M8 && version != VER_M9 && version != VER_M10) {
      version = VER_INVALID;
      break;
    }

    ubx_cfg_prt_t cfg_prt = {
        .portID = 0,
        .reserved1 = 0,
        .txReady = 0,
        .mode = 0xac0,
        .baudRate = UBX_BAUDRATE,
        .inProtoMask = 0b11,
        .outProtoMask = 0b01,
        .flags = 0,
        .reserved2 = 0,
    };
    gps_send_message(UBX_CFG_PRT, (uint8_t *)&cfg_prt, sizeof(cfg_prt));
    break;
  }
  case UBX_ACK_ACK:
    break;
  case UBX_ACK_NAK:
    break;
  default:
    break;
  }
}

static bool gps_parse_ubx(uint8_t data) {
  static gps_parser_state_t state = SYNC1;
  static uint16_t class_id = 0;
  static uint16_t offset = 0;
  static uint16_t length = 0;
  static uint8_t payload[256];

  static uint8_t ck_a = 0;
  static uint8_t ck_b = 0;

  switch (state) {
  case SYNC1:
    if (data == UBX_SYNC1)
      state = SYNC2;
    break;
  case SYNC2:
    if (data == UBX_SYNC2) {
      ck_a = 0;
      ck_b = 0;
      state = CLASS;
    } else
      state = SYNC1;
    break;
  case CLASS:
    gps_crc(&ck_a, &ck_b, data);
    class_id = data << 8;
    state = ID;
    break;
  case ID:
    gps_crc(&ck_a, &ck_b, data);
    class_id |= data;
    offset = 0;
    state = LENGTH;
    break;
  case LENGTH:
    gps_crc(&ck_a, &ck_b, data);
    if (offset == 0) {
      length = data;
      offset++;
      break;
    }
    if (offset == 1) {
      offset = 0;
      length |= data << 8;
      state = PAYLOAD;
      break;
    }
    break;
  case PAYLOAD:
    gps_crc(&ck_a, &ck_b, data);
    if (offset < 128)
      payload[offset] = data;
    offset++;
    if (offset == length) {
      offset = 0;
      state = CHECKSUM;
    }
    break;
  case CHECKSUM:
    if (offset == 0) {
      if (ck_a != data)
        state = SYNC1;
      offset++;
      break;
    }
    if (offset == 1) {
      state = SYNC1;
      if (ck_b == data) {
        gps_handle_packet(class_id, payload, length);
        return true;
      }
    }
    break;
  }

  return false;
}

void gps_init() {
  if (profile.serial.gps == SERIAL_PORT_INVALID)
    return;

  serial_port_config_t config;
  config.port = profile.serial.gps;
  config.baudrate = UBX_BAUDRATE;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = false;
  config.half_duplex_pp = false;
  serial_init(&serial_gps, config);
}

bool gps_update() {
  if (profile.serial.gps == SERIAL_PORT_INVALID)
    return false;

  uint8_t byte = 0;
  while (serial_read_byte(&serial_gps, &byte))
    gps_parse_ubx(byte);

  static uint8_t last_version_check = 0;
  if (version == VER_INVALID && time_millis() - last_version_check > 10) {
    gps_send_message(UBX_MON_VER, NULL, 0);
    last_version_check = time_millis();
  }
  return false;
}
#else
void gps_init(void) {}
bool gps_update(void) { return false; }
#endif
