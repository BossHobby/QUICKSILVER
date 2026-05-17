#include "gps.h"

#include <stdlib.h>
#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "util/cbor_helper.h"
#include "util/ring_buffer.h"

#ifdef USE_GPS

#define UBX_SYNC1_CHAR 0xb5
#define UBX_SYNC2_CHAR 0x62

#define UBX_MON_VER 0x0a04
#define UBX_NAV_PVT 0x0107
#define UBX_NAV_SAT 0x0135
#define UBX_AID_INI 0x0b01
#define UBX_CFG_PRT 0x0600
#define UBX_CFG_MSG 0x0601
#define UBX_CFG_RATE 0x0608
#define UBX_CFG_NAV5 0x0624
#define UBX_CFG_SBAS 0x0616
#define UBX_CFG_VALGET 0x068b
#define UBX_CFG_VALSET 0x068a
#define UBX_ACK_ACK 0x0501
#define UBX_ACK_NAK 0x0500

// Configuration keys for M10
// Note: Performance mode changes require a power cycle to take effect
// M10 modules support 25Hz with all constellations only in performance mode
#define CFG_PM_OPERATEMODE 0x20d00001   // Power mode: 0=Full power, 1=Balanced, 2=Low power
#define CFG_SIGNAL_GPS_ENA 0x1031001f   // GPS enable
#define CFG_SIGNAL_GLO_ENA 0x10310025   // GLONASS enable
#define CFG_SIGNAL_GAL_ENA 0x10310021   // Galileo enable
#define CFG_SIGNAL_BDS_ENA 0x10310022   // BeiDou enable

// Ground assistance configuration keys
#define CFG_NAVSPG_UTCSTANDARD 0x2011001c // UTC standard (0=auto)
#define CFG_NAVSPG_DYNMODEL 0x20110021    // Dynamic platform model
#define CFG_NAVSPG_FIXMODE 0x20110011     // Position fix mode

#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007 // UBX-NAV-PVT output rate on UART1
#define CFG_MSGOUT_UBX_NAV_SAT_UART1 0x20910016 // UBX-NAV-SAT output rate on UART1

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

typedef struct {
  uint8_t version;
  uint8_t layer;
  uint8_t reserved[2];
  uint32_t cfgDataKey;
} ubx_cfg_valget_t;

// Satellite info for NAV-SAT message
typedef struct {
  uint8_t gnssId; // GNSS identifier (0=GPS, 1=SBAS, 2=Galileo, 3=BeiDou, 5=QZSS, 6=GLONASS)
  uint8_t svId;   // Satellite identifier
  uint8_t cno;    // Carrier to noise ratio (dBHz)
  int8_t elev;    // Elevation (deg)
  int16_t azim;   // Azimuth (deg)
  int16_t prRes;  // Pseudorange residual (0.1m)
  uint32_t flags; // Flags
} ubx_nav_sat_info_t;

typedef struct {
  uint32_t iTOW;   // GPS time of week [ms]
  uint8_t version; // Message version (1)
  uint8_t numSvs;  // Number of satellites
  uint8_t reserved1[2];
  ubx_nav_sat_info_t sats[]; // Variable length array
} ubx_nav_sat_t;

// Position aiding structure
typedef struct {
  int32_t ecefXorLat;        // ECEF X coordinate or latitude [1e-7 deg or cm]
  int32_t ecefYorLon;        // ECEF Y coordinate or longitude [1e-7 deg or cm]
  int32_t ecefZorAlt;        // ECEF Z coordinate or altitude [cm]
  uint32_t posAcc;           // Position accuracy [cm]
  uint16_t tmCfg;            // Time mark configuration
  uint16_t wnoOrDate;        // GPS week number or date
  uint32_t towOrTime;        // GPS time of week or time [ms or s]
  int32_t towNs;             // Sub-millisecond TOW [ns]
  uint32_t tAccMs;           // Time accuracy [ms]
  uint32_t tAccNs;           // Sub-ms time accuracy [ns]
  int32_t clkDOrFreq;        // Clock drift or frequency [ns/s or Hz]
  uint32_t clkDAccOrFreqAcc; // Accuracy of clock drift/frequency
  uint32_t flags;            // Configuration flags
} ubx_aid_ini_t;

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

static gps_ubx_parser_state_t ubx_parser_state = UBX_SYNC1;

static bool is_init = false;
static bool had_ack = false;
static bool had_nak = false;

static uint8_t desired_baudrate = 0;
static uint8_t next_baudrate = 0;
static uint8_t current_baudrate = 0;

static bool performance_mode_checked = false;
static uint8_t current_power_mode = 0xFF; // 0xFF = unknown
static uint8_t constellation_index = 0;

// GPS status tracking
gps_status_t gps_status = {
    .state = GPS_DETECT_BAUD,
    .version = VER_INVALID,
    .fix_type = GPS_FIX_NONE,
    .power_mode = GPS_POWER_UNKNOWN,
};

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
  if (serial_bytes_free(&serial_gps) < 8 + size)
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

// Helper function to send CFG-VALSET message
static void gps_send_cfg_valset(uint32_t key, uint8_t value, uint8_t layer) {
  if (gps_status.version == VER_M9 || gps_status.version == VER_M10) {
    const uint8_t cfg_valset[] = {
        0,
        layer,
        0,
        0,
        key & 0xff,
        (key >> 8) & 0xff,
        (key >> 16) & 0xff,
        (key >> 24) & 0xff,
        value,
    };
    gps_send_message(UBX_CFG_VALSET, cfg_valset, sizeof(cfg_valset));
  }
}

static void gps_reset_ack_state() {
  had_ack = false;
  had_nak = false;
}

static bool gps_had_ack_response() {
  return had_ack || had_nak;
}

static void gps_set_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate, uint32_t cfg_key) {
  const uint8_t cfg_msg[] = {
      msg_class,
      msg_id,
      0,
      rate,
      0,
      0,
      0,
      0,
  };
  gps_send_message(UBX_CFG_MSG, cfg_msg, sizeof(cfg_msg));
  gps_send_cfg_valset(cfg_key, rate, 0x01);
}

static void gps_set_nav_pvt_messages(bool enable) {
  gps_set_message_rate(0x01, 0x07, enable ? 1 : 0, CFG_MSGOUT_UBX_NAV_PVT_UART1);
}

// Helper function to enable/disable NAV-SAT messages
static void gps_set_nav_sat_messages(bool enable) {
  gps_set_message_rate(0x01, 0x35, enable ? 5 : 0, CFG_MSGOUT_UBX_NAV_SAT_UART1);
}

// Helper function to update dynamic model
static void gps_update_dynamic_model(bool should_be_static) {
  uint8_t new_dyn_model = should_be_static ? 2 : 8; // 2=Stationary, 8=Airborne<4g
  uint8_t new_static_hold = 0;

  if (gps_status.version == VER_M9 || gps_status.version == VER_M10) {
    gps_send_cfg_valset(CFG_NAVSPG_DYNMODEL, new_dyn_model, 0x01);
  } else {
    // Use CFG-NAV5 for M8
    const ubx_cfg_nav5_t cfg_nav5 = {
        .mask = 0x0005, // Only update dynModel and staticHoldThresh
        .dynModel = new_dyn_model,
        .fixMode = 3,
        .fixedAlt = 0,
        .fixedAltVar = 10000,
        .minElev = 5,
        .drLimit = 0,
        .pDop = 250,
        .tDop = 250,
        .pAcc = 100,
        .tAcc = 300,
        .staticHoldThresh = new_static_hold,
        .dgnssTimeout = 60,
        .cnoThreshNumSVs = 0,
        .cnoThresh = 0,
        .staticHoldMaxDist = 200,
        .utcStandard = 0,
    };
    gps_send_message(UBX_CFG_NAV5, (uint8_t *)&cfg_nav5, sizeof(cfg_nav5));
  }
}

static void gps_handle_packet(const uint16_t class_id, const uint8_t *payload, const uint16_t size) {
  switch (class_id) {
  case UBX_NAV_PVT: {
    if (size < sizeof(ubx_nav_pvt_t))
      break;

    ubx_nav_pvt_t *nav_pvt = (ubx_nav_pvt_t *)payload;

    // Update basic state
    state.gps_lock = (nav_pvt->flags & FLAGS_GNSS_FIX_OK) && (nav_pvt->numSV >= GPS_MIN_SATS_FOR_LOCK);
    state.gps_sats = nav_pvt->numSV;
    state.gps_coord.lon = nav_pvt->lon;
    state.gps_coord.lat = nav_pvt->lat;
    state.gps_altitude = nav_pvt->hMSL / 1000.0f;
    state.gps_heading = nav_pvt->headMot / 100000.0f;
    state.gps_heading_accuracy = nav_pvt->headAcc / 100000.0f;
    state.gps_horizontal_accuracy = nav_pvt->hAcc * 0.001f;
    state.gps_last_update_ms = time_millis();
    state.gps_speed = nav_pvt->gSpeed * 0.001f; // mm/s to m/s
    state.gps_vel_north = nav_pvt->velN * 0.001f;
    state.gps_vel_east = nav_pvt->velE * 0.001f;

    // Update comprehensive status
    gps_status.fix_type = nav_pvt->fixType;
    gps_status.sats_used = nav_pvt->numSV;
    gps_status.pdop = nav_pvt->pDOP;
    gps_status.h_acc = nav_pvt->hAcc;
    gps_status.v_acc = nav_pvt->vAcc;

    // Update fix quality (0-100%)
    if (nav_pvt->fixType >= GPS_FIX_3D) {
      // Calculate quality based on satellites, DOP, and accuracy
      uint8_t sat_quality = (nav_pvt->numSV > 12) ? 40 : (nav_pvt->numSV * 40 / 12);
      uint8_t dop_quality = (nav_pvt->pDOP < 200) ? 30 : (30 * 200 / nav_pvt->pDOP);
      uint8_t acc_quality = (nav_pvt->hAcc < 5000) ? 30 : (30 * 5000 / nav_pvt->hAcc);
      gps_status.fix_quality = sat_quality + dop_quality + acc_quality;
    } else {
      gps_status.fix_quality = 0;
    }
    break;
  }
  case UBX_NAV_SAT: {
    if (size < sizeof(ubx_nav_sat_t))
      break;

    ubx_nav_sat_t *nav_sat = (ubx_nav_sat_t *)payload;
    const uint8_t sat_count = min(nav_sat->numSvs, (size - sizeof(ubx_nav_sat_t)) / sizeof(ubx_nav_sat_info_t));

    // Reset constellation counts
    gps_status.gps = (gps_constellation_t){0};
    gps_status.glonass = (gps_constellation_t){0};
    gps_status.galileo = (gps_constellation_t){0};
    gps_status.beidou = (gps_constellation_t){0};

    gps_status.sats_in_view = sat_count;
    uint32_t total_cno = 0;
    uint8_t cno_count = 0;

    // Process each satellite
    for (uint8_t i = 0; i < sat_count; i++) {
      ubx_nav_sat_info_t *sat = &nav_sat->sats[i];

      // Track signal quality
      if (sat->cno > 0) {
        total_cno += sat->cno;
        cno_count++;
      }

      // Update constellation-specific info
      gps_constellation_t *constellation = NULL;
      switch (sat->gnssId) {
      case 0: // GPS
        constellation = &gps_status.gps;
        break;
      case 6: // GLONASS
        constellation = &gps_status.glonass;
        break;
      case 2: // Galileo
        constellation = &gps_status.galileo;
        break;
      case 3: // BeiDou
        constellation = &gps_status.beidou;
        break;
      }

      if (constellation) {
        constellation->sats_in_view++;
        constellation->enabled = 1;

        // Check if satellite is used in solution (bit 3 of flags)
        if (sat->flags & 0x08) {
          constellation->sats_used++;
        }

        // Constellation is healthy if we have good signals
        if (sat->cno > 30) {
          constellation->healthy = 1;
        }
      }
    }

    // Calculate average CNO
    if (cno_count > 0) {
      gps_status.avg_cno = total_cno / cno_count;
    } else {
      gps_status.avg_cno = 0;
    }
    break;
  }
  case UBX_MON_VER: {
    if (size < 40)
      break;

    const uint32_t version = strtoul((const char *)(payload + 30), NULL, 16);
    if (version != VER_M8 && version != VER_M9 && version != VER_M10) {
      gps_status.version = VER_INVALID;
      gps_status.state = GPS_NOT_DETECTED;
    } else {
      gps_status.version = version;
    }
    break;
  }
  case UBX_CFG_VALGET: {
    // Response to our power mode query
    if (size >= 8) { // version(1) + layer(1) + reserved(2) + cfgDataKey(4)
      // Key is little-endian in the message
      uint32_t key = payload[4] | (payload[5] << 8) | (payload[6] << 16) | (payload[7] << 24);
      if (key == CFG_PM_OPERATEMODE && size >= 9) {
        current_power_mode = payload[8]; // 0 = full power, 1 = balanced, 2 = low power
        gps_status.power_mode = current_power_mode;
        performance_mode_checked = true;
      }
    }
    break;
  }
  case UBX_ACK_ACK:
    had_ack = true;
    break;
  case UBX_ACK_NAK:
    had_nak = true;
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
      if (length > sizeof(payload)) {
        ubx_parser_state = UBX_SYNC1;
        break;
      }
      if (length == 0) {
        ubx_parser_state = UBX_CHECKSUM;
        break;
      }
      ubx_parser_state = UBX_PAYLOAD;
      break;
    }
    break;
  case UBX_PAYLOAD:
    gps_crc(&ck_a, &ck_b, data);
    if (offset < sizeof(payload))
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
    gps_status.version = VER_INVALID;
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
  bool had_update = false;

  uint8_t byte = 0;
  WHILE_TIMEOUT(serial_read_byte(&serial_gps, &byte), 500) {
    if (gps_parse_ubx(byte))
      had_update = true;
  }

  return had_update;
}

void gps_task() {
  if (!is_init)
    return;

  static uint32_t last_baud_rate_change = 0;
  static uint32_t config_request_time = 0;

  switch (gps_status.state) {
  case GPS_DETECT_BAUD: {
    if (gps_status.version != VER_INVALID) {
      next_baudrate = desired_baudrate;
      serial_write_bytes(&serial_gps, (const uint8_t *)baudrates[next_baudrate].pubx, strlen(baudrates[next_baudrate].pubx));
      last_baud_rate_change = time_millis();
      gps_status.state = GPS_CHANGE_BAUD;
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

      // For M10, check performance mode before setting rate
      if (gps_status.version == VER_M10) {
        // Query current power mode
        const ubx_cfg_valget_t cfg_valget = {
            .version = 0,
            .layer = 0, // RAM layer
            .reserved = {0, 0},
            .cfgDataKey = CFG_PM_OPERATEMODE,
        };
        gps_send_message(UBX_CFG_VALGET, (uint8_t *)&cfg_valget, sizeof(cfg_valget));
        gps_status.state = GPS_CHECK_PERFORMANCE;
        performance_mode_checked = false;
        current_power_mode = 0xFF; // Reset to unknown
        config_request_time = time_millis();
      } else {
        // For M8/M9, skip performance check but go through constellation config
        gps_status.state = GPS_CONFIG_CONSTELLATIONS;
      }
      gps_reset_ack_state();
    }
    break;
  }
  case GPS_CHECK_PERFORMANCE: {
    gps_read_ublox();

    if (performance_mode_checked) {
      bool need_performance_mode = (current_power_mode != 0);

      if (need_performance_mode) {
        // Set to full power (high performance) mode
        // NOTE: This change requires a power cycle to take effect!
        gps_send_cfg_valset(CFG_PM_OPERATEMODE, 0, 0x07);
      }

      // Move to constellation config
      gps_status.state = GPS_CONFIG_CONSTELLATIONS;
    }
    if ((time_millis() - config_request_time) > 500) {
      gps_status.state = GPS_CONFIG_CONSTELLATIONS;
    }
    break;
  }
  case GPS_CONFIG_CONSTELLATIONS: {
    // Only M9 and M10 support CFG-VALSET for constellation configuration
    if (gps_status.version == VER_M9 || gps_status.version == VER_M10) {
      // Enable all GNSS constellations for best accuracy
      // Only enable if in performance mode (M10) or always for M9
      if (gps_status.version == VER_M9 || current_power_mode == 0) {
        const struct {
          uint32_t key;
          uint8_t value;
        } gnss_config[] = {
            {CFG_SIGNAL_GPS_ENA, 1}, // GPS
            {CFG_SIGNAL_GLO_ENA, 1}, // GLONASS
            {CFG_SIGNAL_GAL_ENA, 1}, // Galileo
            {CFG_SIGNAL_BDS_ENA, 1}, // BeiDou
        };

        if (constellation_index < sizeof(gnss_config) / sizeof(gnss_config[0])) {
          gps_send_cfg_valset(gnss_config[constellation_index].key, gnss_config[constellation_index].value, 0x01);
          constellation_index++;
          break; // Stay in this state until all constellations are configured
        }
      }
    }

    gps_status.state = GPS_CONFIG_UPDATE_RATE;
    break;
  }
  case GPS_CONFIG_UPDATE_RATE: {
    // Match Betaflight's conservative default. Higher rates can increase UART load before lock.
    const uint16_t rate = 100; // 10Hz

    const ubx_cfg_rate_t cfg_rate = {
        .measRate = rate,
        .navRate = 1,
        .timeRef = 1,
    };
    gps_send_message(UBX_CFG_RATE, (uint8_t *)&cfg_rate, sizeof(cfg_rate));
    config_request_time = time_millis();

    // Update status with configured rate
    gps_status.update_rate = 1000 / rate;

    gps_status.state = GPS_CONFIG_NAV_MODEL;
    gps_reset_ack_state();
    break;
  }
  case GPS_CONFIG_NAV_MODEL: {
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      // Configure for optimal acquisition on ground vs in flight
      uint8_t dyn_model = 8; // Default: Airborne < 4g

      // If on ground and not moving, use static model for faster lock
      if (!flags.arm_state && state.gps_speed < 1.0f) {
        dyn_model = 2; // Stationary model
      }

      const ubx_cfg_nav5_t cfg_nav5 = {
          .mask = 0xFFFF,
          .dynModel = dyn_model,
          .fixMode = 3, // 3 = Auto 2D/3D
          .fixedAlt = 0,
          .fixedAltVar = 10000,
          .minElev = 5, // 5 degrees minimum elevation to reduce multipath
          .drLimit = 0, // No dead reckoning
          .pDop = 250,
          .tDop = 250,
          .pAcc = 100,
          .tAcc = 300,
          .staticHoldThresh = 0,
          .dgnssTimeout = 60,
          .cnoThreshNumSVs = 0,     // No minimum satellites required
          .cnoThresh = 0,           // Accept all signal strengths
          .staticHoldMaxDist = 200, // 200m static hold distance
          .utcStandard = 0,
      };
      gps_send_message(UBX_CFG_NAV5, (uint8_t *)&cfg_nav5, sizeof(cfg_nav5));
      config_request_time = time_millis();
      gps_status.state++;
      gps_reset_ack_state();
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_CONFIG_SBAS: {
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      const ubx_cfg_sbas_t cfg_sbas = {
          .mode = 0b10,
          .usage = 0b011,
          .maxSBAS = 3,
          .scanmode1 = 0,
          .scanmode2 = 0,
      };
      gps_send_message(UBX_CFG_SBAS, (uint8_t *)&cfg_sbas, sizeof(cfg_sbas));
      config_request_time = time_millis();
      gps_status.state++;
      gps_reset_ack_state();
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_CONFIG_MESSAGES: {
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      // Ensure the configured UART outputs the navigation messages used by the firmware.
      gps_set_nav_pvt_messages(true);
      gps_set_nav_sat_messages(true);
      gps_status.state = GPS_WAITING_FOR_LOCK;
      gps_reset_ack_state();
      break;
    }
    gps_read_ublox();
    break;
  }
  case GPS_WAITING_FOR_LOCK: {
    gps_read_ublox();

    if (state.gps_lock) {
      gps_update_dynamic_model(false);
      config_request_time = time_millis();
      gps_reset_ack_state();
      gps_status.state = GPS_CONFIG_FLIGHT_MODEL;
      break;
    }

    // Note: Modern GPS modules with BBR automatically store and use last position for faster reacquisition
    break;
  }
  case GPS_CONFIG_FLIGHT_MODEL: {
    gps_read_ublox();

    if (!state.gps_lock) {
      gps_status.state = GPS_WAITING_FOR_LOCK;
      break;
    }

    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      gps_status.state = GPS_RUNNING;
      gps_reset_ack_state();
    }
    break;
  }
  case GPS_RUNNING: {
    gps_read_ublox();

    // Check if we lost lock
    if (!state.gps_lock) {
      gps_status.state = GPS_WAITING_FOR_LOCK;
      break;
    }

    if (flags.arm_state) {
      gps_set_nav_sat_messages(false);
      config_request_time = time_millis();
      gps_reset_ack_state();
      gps_status.state = GPS_CONFIG_DISABLE_NAV_SAT;
    }
    break;
  }
  case GPS_CONFIG_DISABLE_NAV_SAT: {
    gps_read_ublox();

    if (!state.gps_lock) {
      gps_status.state = GPS_WAITING_FOR_LOCK;
      break;
    }

    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      gps_status.state = GPS_RUNNING_NAV_SAT_OFF;
      gps_reset_ack_state();
    }
    break;
  }
  case GPS_RUNNING_NAV_SAT_OFF: {
    gps_read_ublox();

    if (!state.gps_lock) {
      gps_status.state = GPS_WAITING_FOR_LOCK;
    }
    break;
  }
  default:
    break;
  }
}

// CBOR encoding functions
#define MEMBER CBOR_ENCODE_MEMBER
#define START_STRUCT CBOR_START_STRUCT_ENCODER
#define END_STRUCT CBOR_END_STRUCT_ENCODER
GPS_CONSTELLATION_MEMBERS
#undef MEMBER
#undef START_STRUCT
#undef END_STRUCT

#define MEMBER CBOR_ENCODE_MEMBER
#define START_STRUCT CBOR_START_STRUCT_ENCODER
#define END_STRUCT CBOR_END_STRUCT_ENCODER
GPS_STATUS_MEMBERS
#undef MEMBER
#undef START_STRUCT
#undef END_STRUCT

#else
void gps_init(void) {}
void gps_task(void) {}
#endif
