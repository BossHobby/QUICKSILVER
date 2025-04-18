#include "gps.h"

#include <stdlib.h>
#include <string.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "flight/control.h"
#include "util/ring_buffer.h"

#define UBX_SYNC1_CHAR 0xb5
#define UBX_SYNC2_CHAR 0x62

#define UBX_MON_VER 0x0a04
#define UBX_NAV_PVT 0x0107
#define UBX_CFG_PRT 0x0600
#define UBX_CFG_RATE 0x0608
#define UBX_CFG_NAV5 0x0624
#define UBX_CFG_SBAS 0x0616
#define UBX_ACK_ACK 0x0501
#define UBX_ACK_NAK 0x0500

typedef enum {
  GPS_DETECT_BAUD,
  GPS_CHANGE_BAUD,
  GPS_CONFIG_RATE,
  GPS_CONFIG_NAV5,
  GPS_CONFIG_SBAS,

  GPS_RUNNING,
  GPS_NOT_DETECTED,
} gps_config_state_t;

typedef struct {
  uint32_t baud;
  const char *pubx;
} gps_baudrate_t;

typedef enum {
  UBX_SYNC1,
  UBX_SYNC2,
  UBX_CLASS,
  UBX_ID,
  UBX_LENGTH,
  UBX_PAYLOAD,
  UBX_CHECKSUM,
} gps_ubx_parser_state_t;

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

typedef struct {
  uint16_t measRate;
  uint16_t navRate;
  uint16_t timeRef;
} ubx_cfg_rate_t;

typedef struct {
  uint16_t mask;              // Parameters bitmask
  uint8_t dynModel;           // Dynamic platform model
  uint8_t fixMode;            // Position fixing mode
  int32_t fixedAlt;           // Fixed altitude for 2D fix mode [cm]
  uint32_t fixedAltVar;       // Fixed altitude variance [cm^2]
  int8_t minElev;             // Minimum elevation for a GNSS satellite [deg]
  uint8_t drLimit;            // Max time to perform dead reckoning [s]
  uint16_t pDop;              // Position DOP mask [0.1]
  uint16_t tDop;              // Time DOP mask [0.1]
  uint16_t pAcc;              // Position accuracy mask [m]
  uint16_t tAcc;              // Time accuracy mask [m]
  uint8_t staticHoldThresh;   // Static hold threshold [cm/s]
  uint8_t dgnssTimeout;       // DGNSS timeout [s]
  uint8_t cnoThreshNumSVs;    // Number of satellites required above CNO mask
  uint8_t cnoThresh;          // CNO threshold for deciding satellites [dBHz]
  uint8_t reserved0[2];       // Reserved
  uint16_t staticHoldMaxDist; // Static hold max distance [m]
  uint8_t utcStandard;        // UTC standard to be used
  uint8_t reserved1[5];       // Reserved
} ubx_cfg_nav5_t;

typedef struct {
  uint8_t mode;
  uint8_t usage;
  uint8_t maxSBAS;
  uint8_t scanmode2;
  uint32_t scanmode1;
} ubx_cfg_sbas_t;

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

static gps_config_state_t config_state = GPS_DETECT_BAUD;
static gps_ubx_parser_state_t ubx_parser_state = UBX_SYNC1;

static bool is_init = false;
static bool had_ack = false;

static uint8_t desired_baudrate = 0;
static uint8_t next_baudrate = 0;
static uint8_t current_baudrate = 0;

static const gps_baudrate_t baudrates[] = {
    {115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n"},
    {57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n"},
    {38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n"},
    {19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n"},
    {9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n"},
};

static void gps_crc(uint8_t *ck_a, uint8_t *ck_b, uint8_t data) {
  *ck_a = *ck_a + data;
  *ck_b = *ck_b + *ck_a;
}

static bool gps_send_message(const uint16_t class_id, const uint8_t *payload, const uint16_t size) {
  if (serial_bytes_free(&serial_gps) < 5 + 2 + size)
    return false;

  const uint8_t header[] = {
      UBX_SYNC1_CHAR,
      UBX_SYNC2_CHAR,
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
    state.gps_lock = nav_pvt->flags & FLAGS_GNSS_FIX_OK;
    state.gps_sats = nav_pvt->numSV;
    state.gps_coord.lon = nav_pvt->lon;
    state.gps_coord.lat = nav_pvt->lat;
    state.gps_altitude = nav_pvt->hMSL / 1000.0f;
    state.gps_heading = nav_pvt->headMot / 100000.0f;
    state.gps_heading_accuracy = nav_pvt->headAcc / 100000.0f;
    state.gps_speed = nav_pvt->gSpeed * 0.0036f;
    break;
  }
  case UBX_MON_VER: {
    const uint32_t version = strtoul((const char *)(payload + 30), NULL, 16);
    if (version != VER_M8 && version != VER_M9 && version != VER_M10) {
      state.gps_version = VER_INVALID;
      config_state = GPS_NOT_DETECTED;
    } else {
      state.gps_version = version;
    }
    break;
  }
  case UBX_ACK_ACK:
    had_ack = true;
    break;
  case UBX_ACK_NAK:
    break;
  default:
    break;
  }
}

static bool gps_parse_ubx(uint8_t data) {
  static uint16_t class_id = 0;
  static uint16_t offset = 0;
  static uint16_t length = 0;
  static uint8_t payload[256];

  static uint8_t ck_a = 0;
  static uint8_t ck_b = 0;

  switch (ubx_parser_state) {
  case UBX_SYNC1:
    if (data == UBX_SYNC1_CHAR)
      ubx_parser_state = UBX_SYNC2;
    break;
  case UBX_SYNC2:
    if (data == UBX_SYNC2_CHAR) {
      ck_a = 0;
      ck_b = 0;
      ubx_parser_state = UBX_CLASS;
    } else
      ubx_parser_state = UBX_SYNC1;
    break;
  case UBX_CLASS:
    gps_crc(&ck_a, &ck_b, data);
    class_id = data << 8;
    ubx_parser_state = UBX_ID;
    break;
  case UBX_ID:
    gps_crc(&ck_a, &ck_b, data);
    class_id |= data;
    offset = 0;
    ubx_parser_state = UBX_LENGTH;
    break;
  case UBX_LENGTH:
    gps_crc(&ck_a, &ck_b, data);
    if (offset == 0) {
      length = data;
      offset++;
      break;
    }
    if (offset == 1) {
      offset = 0;
      length |= data << 8;
      ubx_parser_state = UBX_PAYLOAD;
      break;
    }
    break;
  case UBX_PAYLOAD:
    gps_crc(&ck_a, &ck_b, data);
    if (offset < 128)
      payload[offset] = data;
    offset++;
    if (offset == length) {
      offset = 0;
      ubx_parser_state = UBX_CHECKSUM;
    }
    break;
  case UBX_CHECKSUM:
    if (offset == 0) {
      if (ck_a != data)
        ubx_parser_state = UBX_SYNC1;
      offset++;
      break;
    }
    if (offset == 1) {
      ubx_parser_state = UBX_SYNC1;
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
  if (profile.serial.gps == SERIAL_PORT_INVALID) {
    is_init = false;
    return;
  }

  serial_port_config_t config;
  config.port = profile.serial.gps;
  config.baudrate = baudrates[next_baudrate].baud;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = false;
  config.half_duplex_pp = false;
  serial_init(&serial_gps, config);

  is_init = true;
  ubx_parser_state = UBX_SYNC1;
  current_baudrate = next_baudrate;
}

static bool gps_read_ublox() {
  uint8_t byte = 0;
  WHILE_TIMEOUT(serial_read_byte(&serial_gps, &byte), 500) {
    if (gps_parse_ubx(byte))
      return true;
  }
  return false;
}

bool gps_update() {
  bool had_update = false;
  if (!is_init)
    return had_update;

  static uint32_t last_baud_rate_change = 0;

  switch (config_state) {
  case GPS_DETECT_BAUD: {
    if (state.gps_version != VER_INVALID) {
      next_baudrate = desired_baudrate;
      serial_write_bytes(&serial_gps, (const uint8_t *)baudrates[next_baudrate].pubx, strlen(baudrates[next_baudrate].pubx));
      last_baud_rate_change = time_millis();
      config_state = GPS_CHANGE_BAUD;
      break;
    }

    gps_read_ublox();

    if (serial_gps.tx_done) {
      static uint32_t last_version_check = 0;
      if ((time_millis() - last_baud_rate_change) > 500) {
        next_baudrate = (next_baudrate + 1) % (sizeof(baudrates) / sizeof(baudrates[0]));
        gps_init();
        last_baud_rate_change = time_millis();
        last_version_check = time_millis();
        break;
      }
      if ((time_millis() - last_version_check) > 50) {
        gps_send_message(UBX_MON_VER, NULL, 0);
        last_version_check = time_millis();
        break;
      }
    }
    break;
  }
  case GPS_CHANGE_BAUD: {
    if ((time_millis() - last_baud_rate_change) > 1500) {
      gps_init();

      const ubx_cfg_rate_t cfg_rate = {
          .measRate = 100,
          .navRate = 1,
          .timeRef = 1,
      };
      gps_send_message(UBX_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(cfg_rate));

      config_state++;
      had_ack = false;
    }
    break;
  }
  case GPS_CONFIG_RATE: {
    if (had_ack) {
      const ubx_cfg_nav5_t cfg_nav5 = {
          .mask = 0xFFFF,
          .dynModel = 8,
          .fixMode = 3,
          .fixedAlt = 0,
          .fixedAltVar = 10000,
          .minElev = 5,
          .drLimit = 0,
          .pDop = 250,
          .tDop = 250,
          .pAcc = 100,
          .tAcc = 300,
          .staticHoldThresh = 0,
          .dgnssTimeout = 60,
          .cnoThreshNumSVs = 0,
          .cnoThresh = 0,
          .staticHoldMaxDist = 200,
          .utcStandard = 0,
      };
      gps_send_message(UBX_CFG_NAV5, (uint8_t *)&cfg_nav5, sizeof(cfg_nav5));
      config_state++;
      had_ack = false;
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_CONFIG_NAV5: {
    if (had_ack) {
      const ubx_cfg_sbas_t cfg_sbas = {
          .mode = 0b11,
          .usage = 0b111,
          .maxSBAS = 3,
          .scanmode1 = 0,
          .scanmode2 = 0,
      };
      gps_send_message(UBX_CFG_SBAS, (uint8_t *)&cfg_sbas, sizeof(cfg_sbas));
      config_state++;
      had_ack = false;
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_CONFIG_SBAS: {
    if (had_ack) {
      config_state++;
      had_ack = false;
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_RUNNING: {
    had_update = gps_read_ublox();
    break;
  }
  default:
    break;
  }

  return had_update;
}