#include "gps.h"

#include <stdlib.h>
#include <string.h>

#include "control/control.h"
#include "core/profile.h"
#include "core/project.h"
#include "core/tasks.h"
#include "driver/serial.h"
#include "driver/time.h"
#include "util/cbor_helper.h"
#include "util/datetime.h"
#include "util/ring_buffer.h"

#ifdef USE_GPS

typedef enum {
  VER_INVALID,
  VER_M5 = 0x00040005,
  VER_M6 = 0x00040007,
  VER_M7 = 0x00070000,
  VER_M8 = 0x00080000,
  VER_M9 = 0x00190000,
  VER_M10 = 0x000A0000,
  VER_MAX = 0xFFFFFFFF,
} gps_version_t;

#define UBX_SYNC1_CHAR 0xb5
#define UBX_SYNC2_CHAR 0x62

// At the 5 ms task period, this exceeds 115200-baud throughput.
#define GPS_RX_BYTES_PER_UPDATE 128U

#define UBX_MON_VER 0x0a04
#define UBX_MON_RF 0x0a38
#define UBX_SEC_SIG 0x2709
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

// Configuration database keys for M9/M10.
#define CFG_PM_OPERATEMODE 0x20d00001 // Power management: 0=Full power
#define CFG_SIGNAL_GPS_ENA 0x1031001f // GPS enable
#define CFG_SIGNAL_GLO_ENA 0x10310025 // GLONASS enable
#define CFG_SIGNAL_GAL_ENA 0x10310021 // Galileo enable
#define CFG_SIGNAL_BDS_ENA 0x10310022 // BeiDou enable
#define CFG_SIGNAL_GPS_L1CA_ENA 0x10310001
#define CFG_SIGNAL_GAL_E1_ENA 0x10310007
#define CFG_SIGNAL_BDS_B1_ENA 0x1031000d
#define CFG_SIGNAL_BDS_B1C_ENA 0x1031000f // M10 only
#define CFG_SIGNAL_GLO_L1_ENA 0x10310018
#define CFG_ITFM_ENABLE 0x1041000d // Interference (jamming) monitor

// Ground assistance configuration keys
#define CFG_NAVSPG_UTCSTANDARD 0x2011001c // UTC standard (0=auto)
#define CFG_NAVSPG_DYNMODEL 0x20110021    // Dynamic platform model
#define CFG_NAVSPG_FIXMODE 0x20110011     // Position fix mode
#define CFG_RATE_MEAS 0x30210001          // Measurement interval, U2 milliseconds
#define CFG_RATE_NAV 0x30210002           // Measurements per navigation solution, U2
#define CFG_RATE_TIMEREF 0x20210003       // Time reference, U1

#define CFG_MSGOUT_UBX_NAV_PVT_UART1 0x20910007 // UBX-NAV-PVT output rate on UART1
#define CFG_MSGOUT_UBX_NAV_SAT_UART1 0x20910016 // UBX-NAV-SAT output rate on UART1
#define CFG_MSGOUT_UBX_MON_RF_UART1 0x2091035a  // UBX-MON-RF output rate on UART1
#define CFG_MSGOUT_UBX_SEC_SIG_UART1 0x20910635 // UBX-SEC-SIG output rate on UART1

#define GPS_SOLUTION_STALE_MS 500U

// A solution delivered later than this behind the fastest one is treated as a
// receiver clock jump (time resolution, week rollover) rather than a delay.
static constexpr int32_t GPS_EPOCH_MAX_DELAY_MS = 1000;
// Interference reports arrive at 1 Hz; older ones no longer describe the signal.
static constexpr uint32_t GPS_INTERFERENCE_STALE_MS = 3000;
// Navigation rate on M8 and before the receiver generation is known.
static constexpr uint8_t GPS_DEFAULT_RATE_HZ = 10;

typedef struct {
  uint32_t baud;
  const char *pubx;
} gps_baudrate_t;

// One configuration database value; the key's size bits select its width.
typedef struct {
  uint32_t key;
  uint32_t value;
} gps_cfg_item_t;

typedef enum {
  CONSTELLATIONS_UNSENT,
  CONSTELLATIONS_SELECTED, // the profile selection, with explicit signals
  CONSTELLATIONS_FALLBACK, // GPS + Galileo after the receiver rejected the selection
} gps_constellation_request_t;

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

/* Valid field bits */
#define VALID_DATE 0x01           // Valid UTC date
#define VALID_TIME 0x02           // Valid UTC time of day
#define VALID_FULLY_RESOLVED 0x04 // UTC time of day has been fully resolved (no seconds uncertainty)

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

static DMA_RAM uint8_t rx_data[512];
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
static uint8_t configured_dynamic_model = 0xff; // acknowledged by the receiver
static uint8_t requested_dynamic_model = 0xff;  // sent, awaiting its acknowledgement

// time_millis() minus NAV-PVT iTOW at the fastest recent delivery.
static bool epoch_offset_valid = false;
static uint32_t epoch_offset_ms = 0;
static uint32_t last_itow = 0;

static gps_constellation_request_t constellation_request = CONSTELLATIONS_UNSENT;
static uint8_t constellations = 0; // selection the receiver was last asked to track
static uint8_t nav_rate_hz = GPS_DEFAULT_RATE_HZ;

static uint32_t jamming_update_ms = 0;
static uint32_t spoofing_update_ms = 0;

// GPS status tracking
gps_status_t gps_status = {
    .state = GPS_DETECT_BAUD,
    .version = VER_INVALID,
    .fix_type = GPS_FIX_NONE,
    .power_mode = GPS_POWER_UNKNOWN,
};

static const gps_baudrate_t baudrates[] = {
    {115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n"},
    {230400, "$PUBX,41,1,0003,0001,230400,0*1C\r\n"},
    {460800, "$PUBX,41,1,0003,0001,460800,0*13\r\n"},
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
  if (serial_bytes_free(&serial_gps) < 8u + size)
    return false;

  const uint8_t header[] = {
      UBX_SYNC1_CHAR,
      UBX_SYNC2_CHAR,
      uint8_t(class_id >> 8),
      uint8_t(class_id & 0xff),
      uint8_t(size & 0xFF),
      uint8_t(size >> 8),
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

// Send M9/M10 configuration values to RAM as one CFG-VALSET transaction: the
// receiver applies all of them or rejects the message with a NAK.
static bool gps_send_cfg_items(const gps_cfg_item_t *items, uint32_t count) {
  if (gps_status.version != VER_M9 && gps_status.version != VER_M10)
    return false;
  uint8_t config[4 + 12 * 8] = {0, 0x01, 0, 0};
  uint32_t size = 4;
  for (uint32_t i = 0; i < count; i++) {
    // Size bits 28-30: 1 = bit and 2 = byte take one byte, 3 two, 4 four.
    const uint32_t size_id = (items[i].key >> 28) & 0x7;
    const uint32_t width = size_id >= 3 ? 1u << (size_id - 2) : 1;
    if (size + 4 + width > sizeof(config))
      return false;
    for (uint32_t b = 0; b < 4; b++)
      config[size++] = uint8_t(items[i].key >> (8 * b));
    for (uint32_t b = 0; b < width; b++)
      config[size++] = uint8_t(items[i].value >> (8 * b));
  }
  return gps_send_message(UBX_CFG_VALSET, config, size);
}

static bool gps_send_cfg_valset(uint32_t key, uint8_t value) {
  const gps_cfg_item_t item = {key, value};
  return gps_send_cfg_items(&item, 1);
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
  if (gps_status.version == VER_M9 || gps_status.version == VER_M10)
    gps_send_cfg_valset(cfg_key, rate);
  else
    gps_send_message(UBX_CFG_MSG, cfg_msg, sizeof(cfg_msg));
}

// Fastest navigation rate the receiver sustains for the tracked selection
// without its optional high-performance mode. M10 rates follow the MAX-M10S
// datasheet (R06) with BeiDou on B1C; the M9 tracks four GNSS at 25 Hz.
static uint8_t gps_nav_rate_for(uint8_t selection) {
  if (gps_status.version == VER_M9)
    return 25;
  if (gps_status.version != VER_M10)
    return GPS_DEFAULT_RATE_HZ;
  switch (__builtin_popcount(selection)) {
  case 1:
    return 18;
  case 2:
    return 10;
  case 3:
    return (selection & GPS_CONSTELLATION_GLONASS) ? 6 : 8;
  default:
    return 4;
  }
}

static void gps_set_update_rate() {
  // Round the interval up so the receiver never runs above its limit.
  const uint16_t interval_ms = (1000 + nav_rate_hz - 1) / nav_rate_hz;
  if (gps_status.version == VER_M9 || gps_status.version == VER_M10) {
    const gps_cfg_item_t config[] = {
        {CFG_RATE_MEAS, interval_ms},
        {CFG_RATE_NAV, 1},
        {CFG_RATE_TIMEREF, 1},
    };
    gps_send_cfg_items(config, 3);
  } else {
    const ubx_cfg_rate_t rate = {.measRate = interval_ms, .navRate = 1, .timeRef = 1};
    gps_send_message(UBX_CFG_RATE, (const uint8_t *)&rate, sizeof(rate));
  }
}

static uint8_t gps_profile_constellations() {
  const uint8_t selection = profile.gps.constellations & 0x0f;
  return selection ? selection : GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GALILEO;
}

// Select the tracked GNSS in RAM; no receiver flash writes. With signals, each
// constellation's L1 signal is set explicitly: the M10 tracks BeiDou B1I only
// without GLONASS and then at 3 Hz with GPS and Galileo, so it uses B1C, which
// combines with GLONASS and allows 8 Hz. Without signals only the
// constellation switches are sent, which every M9/M10 firmware accepts.
static bool gps_set_constellations(uint8_t selection, bool signals) {
  const bool gps = selection & GPS_CONSTELLATION_GPS;
  const bool glonass = selection & GPS_CONSTELLATION_GLONASS;
  const bool galileo = selection & GPS_CONSTELLATION_GALILEO;
  const bool beidou = selection & GPS_CONSTELLATION_BEIDOU;
  const bool m10 = gps_status.version == VER_M10;
  const gps_cfg_item_t config[] = {
      {CFG_SIGNAL_GPS_ENA, gps},
      {CFG_SIGNAL_GLO_ENA, glonass},
      {CFG_SIGNAL_GAL_ENA, galileo},
      {CFG_SIGNAL_BDS_ENA, beidou},
      {CFG_SIGNAL_GPS_L1CA_ENA, gps},
      {CFG_SIGNAL_GLO_L1_ENA, glonass},
      {CFG_SIGNAL_GAL_E1_ENA, galileo},
      {CFG_SIGNAL_BDS_B1_ENA, beidou && !m10},
      {CFG_SIGNAL_BDS_B1C_ENA, beidou && m10}, // last: the M9 has no B1C key
  };
  const uint32_t count = !signals ? 4 : m10 ? 9 : 8;
  if (!gps_send_cfg_items(config, count))
    return false;
  constellations = selection;
  return true;
}

static void gps_set_nav_pvt_messages(bool enable) {
  gps_set_message_rate(0x01, 0x07, enable ? 1 : 0, CFG_MSGOUT_UBX_NAV_PVT_UART1);
}

// Satellite diagnostics at about 2 Hz, whatever the navigation rate.
static void gps_set_nav_sat_messages(bool enable) {
  const uint8_t every = MAX(nav_rate_hz / 2, 1);
  gps_set_message_rate(0x01, 0x35, enable ? every : 0, CFG_MSGOUT_UBX_NAV_SAT_UART1);
}

// Interference reports at 1 Hz. MON-RF carries the jamming state, antenna
// status and noise; on the M10, SEC-SIG adds spoofing detection. The SEC-SIG
// output goes in its own transaction so a rejection leaves MON-RF configured.
static void gps_set_interference_messages() {
  const gps_cfg_item_t rf[] = {
      {CFG_ITFM_ENABLE, 1},
      {CFG_MSGOUT_UBX_MON_RF_UART1, nav_rate_hz},
  };
  gps_send_cfg_items(rf, 2);
  if (gps_status.version == VER_M10)
    gps_send_cfg_valset(CFG_MSGOUT_UBX_SEC_SIG_UART1, nav_rate_hz);
}

// Helper function to update dynamic model
static bool gps_update_dynamic_model(bool should_be_static) {
  const uint8_t new_dyn_model = should_be_static ? 2 : 8; // 2=Stationary, 8=Airborne<4g
  bool sent;

  if (gps_status.version == VER_M9 || gps_status.version == VER_M10) {
    sent = gps_send_cfg_valset(CFG_NAVSPG_DYNMODEL, new_dyn_model);
  } else {
    // Use CFG-NAV5 for M8
    const ubx_cfg_nav5_t cfg_nav5 = {
        .mask = 0x0001, // Only update dynModel; preserve receiver defaults.
        .dynModel = new_dyn_model,
    };
    sent = gps_send_message(UBX_CFG_NAV5, (uint8_t *)&cfg_nav5, sizeof(cfg_nav5));
  }
  if (sent)
    requested_dynamic_model = new_dyn_model;
  return sent;
}

// Stamp a solution by its receiver epoch instead of its delivery. UART
// transfer and IO scheduling only ever delay delivery, so the smallest
// arrival-minus-iTOW offset carries just the clock difference and the
// receiver's fixed output latency. Consecutive stamps are then spaced by the
// measurement interval, and a delayed burst keeps its real age. Each delayed
// delivery raises the offset by 1 ms: far faster than the clocks drift apart,
// yet it shifts a stamp by at most 1 ms however long the delay.
static uint32_t gps_epoch_ms(uint32_t itow, uint32_t arrival_ms) {
  const uint32_t offset = arrival_ms - itow;
  const int32_t delay = int32_t(offset - epoch_offset_ms);
  if (!epoch_offset_valid || delay < 0 || delay > GPS_EPOCH_MAX_DELAY_MS) {
    epoch_offset_ms = offset;
    epoch_offset_valid = true;
  } else if (delay > 0) {
    epoch_offset_ms++;
  }
  return itow + epoch_offset_ms;
}

static void gps_handle_packet(const uint16_t class_id, const uint8_t *payload, const uint16_t size) {
  switch (class_id) {
  case UBX_NAV_PVT: {
    if (size < sizeof(ubx_nav_pvt_t))
      break;

    ubx_nav_pvt_t *nav_pvt = (ubx_nav_pvt_t *)payload;
    // Report the rate the receiver delivers, not the one requested.
    const uint32_t interval_ms = nav_pvt->iTOW - last_itow;
    if (epoch_offset_valid && interval_ms > 0 && interval_ms <= 1000)
      gps_status.update_rate = (1000 + interval_ms / 2) / interval_ms;
    last_itow = nav_pvt->iTOW;
    const uint32_t epoch_ms = gps_epoch_ms(nav_pvt->iTOW, time_millis());

    // Publish one complete solution before Flight can consume navigation data.
    taskENTER_CRITICAL();
    state.gps_lock = (nav_pvt->flags & FLAGS_GNSS_FIX_OK) &&
                     (nav_pvt->fixType == GPS_FIX_3D || nav_pvt->fixType == GPS_FIX_GNSS_DR) &&
                     (nav_pvt->numSV >= GPS_MIN_SATS_FOR_LOCK);
    state.gps_sats = nav_pvt->numSV;
    state.gps_coord.lon = nav_pvt->lon;
    state.gps_coord.lat = nav_pvt->lat;
    state.gps_altitude = nav_pvt->hMSL / 1000.0f;
    state.gps_heading = nav_pvt->headMot / 100000.0f;
    state.gps_heading_accuracy = nav_pvt->headAcc / 100000.0f;
    state.gps_horizontal_accuracy = nav_pvt->hAcc * 0.001f;
    state.gps_speed_accuracy = nav_pvt->sAcc * 0.001f; // mm/s to m/s
    state.gps_last_update_ms = epoch_ms;
    state.gps_speed = nav_pvt->gSpeed * 0.001f; // mm/s to m/s
    state.gps_vel_north = nav_pvt->velN * 0.001f;
    state.gps_vel_east = nav_pvt->velE * 0.001f;
    state.gps_vel_down = nav_pvt->velD * 0.001f;
    taskEXIT_CRITICAL();

    // Update comprehensive status
    const uint8_t time_valid = VALID_DATE | VALID_TIME | VALID_FULLY_RESOLVED;
    if ((nav_pvt->valid & time_valid) == time_valid) {
      const datetime_t utc = {
          .year = nav_pvt->year,
          .month = nav_pvt->month,
          .day = nav_pvt->day,
          .hour = nav_pvt->hour,
          .minute = nav_pvt->min,
          .second = nav_pvt->sec,
          .millisecond = 0,
      };
      // nano is signed and rounds the seconds field, so apply it separately.
      datetime_set_epoch_ms(datetime_to_epoch_ms(&utc) + nav_pvt->nano / 1000000);
    }

    gps_status.fix_type = nav_pvt->fixType;
    gps_status.sats_used = nav_pvt->numSV;
    gps_status.pdop = nav_pvt->pDOP;
    gps_status.height = nav_pvt->height;
    gps_status.vel_n = nav_pvt->velN;
    gps_status.vel_e = nav_pvt->velE;
    gps_status.vel_d = nav_pvt->velD;
    gps_status.h_acc = nav_pvt->hAcc;
    gps_status.v_acc = nav_pvt->vAcc;
    gps_status.speed_acc = nav_pvt->sAcc;
    gps_status.head_acc = nav_pvt->headAcc;

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
    const uint8_t sat_count = MIN(nav_sat->numSvs, (size - sizeof(ubx_nav_sat_t)) / sizeof(ubx_nav_sat_info_t));

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
  case UBX_MON_RF: {
    // Header, then one 24-byte block per RF band; M9/M10 have only L1.
    if (size < 4 + 24 || payload[1] == 0)
      break;
    const uint8_t *block = payload + 4;
    gps_status.antenna_status = block[2];
    gps_status.noise_per_ms = block[12] | (block[13] << 8);
    gps_status.agc_count = block[14] | (block[15] << 8);
    gps_status.cw_jamming = block[16];
    // Where the receiver reports SEC-SIG, that carries jamming instead. The
    // M10 on SPG 5.10 has no SEC-SIG; its MON-RF state then counts.
    if (time_millis() - spoofing_update_ms > GPS_INTERFERENCE_STALE_MS) {
      state.gps_jamming = block[1] & 0x03;
      jamming_update_ms = time_millis();
    }
    break;
  }
  case UBX_SEC_SIG: {
    // Version 1 has separate jamming and spoofing bytes; version 2 packs both.
    uint8_t jamming, spoofing;
    if (size >= 12 && payload[0] == 1) {
      jamming = (payload[4] >> 1) & 0x03;
      spoofing = (payload[8] >> 1) & 0x07;
    } else if (size >= 4 && payload[0] == 2) {
      jamming = (payload[1] >> 1) & 0x03;
      spoofing = (payload[1] >> 4) & 0x07;
    } else {
      break;
    }
    state.gps_jamming = jamming;
    state.gps_spoofing = spoofing <= GPS_SPOOFING_MULTIPLE ? spoofing : GPS_SPOOFING_UNKNOWN;
    jamming_update_ms = spoofing_update_ms = time_millis();
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

  serial_port_config_t config = {};
  config.port = profile.serial.gps;
  config.baudrate = baudrates[next_baudrate].baud;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = false;
  config.half_duplex_pp = false;
  config.dma_mode = SERIAL_DMA_RX;
  serial_init(&serial_gps, config);

  is_init = true;
  ubx_parser_state = UBX_SYNC1;
  current_baudrate = next_baudrate;
  configured_dynamic_model = requested_dynamic_model = 0xff;
  epoch_offset_valid = false;
  constellation_request = CONSTELLATIONS_UNSENT;
  nav_rate_hz = GPS_DEFAULT_RATE_HZ;
}

static bool gps_read_ublox() {
  uint8_t byte = 0;
  // Preserve partial frames between calls and handle at most one packet.
  for (uint32_t count = 0; count < GPS_RX_BYTES_PER_UPDATE && serial_read_byte(&serial_gps, &byte); count++) {
    if (gps_parse_ubx(byte))
      return true;
  }

  return false;
}

TickType_t gps_task() {
  if (!is_init)
    return portMAX_DELAY;

  static uint32_t last_baud_rate_change = 0;
  static uint32_t config_request_time = 0;

  // Keep ground acquisition settings until arming, even after the first fix.
  // Arming without a fix must still select airborne tracking. On disarm,
  // restore the stationary model and satellite diagnostics without a restart.
  // Only an acknowledgement settles the model; otherwise it is sent again.
  if (gps_status.state >= GPS_WAITING_FOR_LOCK && gps_status.state <= GPS_RUNNING_NAV_SAT_OFF &&
      gps_status.state != GPS_CONFIG_FLIGHT_MODEL &&
      configured_dynamic_model != (flags.arm_state ? 8 : 2) && serial_bytes_free(&serial_gps) >= 64) {
    gps_reset_ack_state();
    if (gps_update_dynamic_model(!flags.arm_state)) {
      config_request_time = time_millis();
      gps_status.state = GPS_CONFIG_FLIGHT_MODEL;
    }
  }

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
        gps_send_cfg_valset(CFG_PM_OPERATEMODE, 0);
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
    if (gps_status.version != VER_M9 && gps_status.version != VER_M10) {
      nav_rate_hz = GPS_DEFAULT_RATE_HZ;
      gps_status.state = GPS_CONFIG_UPDATE_RATE;
      break;
    }
    if (constellation_request == CONSTELLATIONS_UNSENT) {
      if (gps_set_constellations(gps_profile_constellations(), true)) {
        constellation_request = CONSTELLATIONS_SELECTED;
        config_request_time = time_millis();
        gps_reset_ack_state();
      }
      break;
    }
    gps_read_ublox();
    // A rejected selection leaves the receiver on its previous constellations.
    if (had_nak && constellation_request == CONSTELLATIONS_SELECTED) {
      if (gps_set_constellations(GPS_CONSTELLATION_GPS | GPS_CONSTELLATION_GALILEO, false)) {
        constellation_request = CONSTELLATIONS_FALLBACK;
        config_request_time = time_millis();
        gps_reset_ack_state();
      }
      break;
    }
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      // Only an ACK confirms the selection. Otherwise the receiver may track
      // all four, so run at the rate it sustains for them.
      nav_rate_hz = gps_nav_rate_for(had_ack ? constellations : 0x0f);
      constellation_request = CONSTELLATIONS_UNSENT;
      gps_status.state = GPS_CONFIG_UPDATE_RATE;
    }
    break;
  }
  case GPS_CONFIG_UPDATE_RATE: {
    gps_set_update_rate();
    config_request_time = time_millis();
    gps_status.update_rate = 0; // measured from the delivered solutions

    gps_status.state = GPS_CONFIG_NAV_MODEL;
    gps_reset_ack_state();
    break;
  }
  case GPS_CONFIG_NAV_MODEL: {
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      gps_update_dynamic_model(!flags.arm_state);
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
          .scanmode2 = 0,
          .scanmode1 = 0,
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
      gps_set_interference_messages();
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
      gps_status.state = GPS_RUNNING;
      break;
    }

    // Note: Modern GPS modules with BBR automatically store and use last position for faster reacquisition
    break;
  }
  case GPS_CONFIG_FLIGHT_MODEL: {
    gps_read_ublox();
    // The model goes alone, so the acknowledgement is its own. Satellite
    // diagnostics follow it: on the ground only.
    if (had_ack) {
      configured_dynamic_model = requested_dynamic_model;
      gps_set_nav_sat_messages(configured_dynamic_model == 2);
    }
    if (gps_had_ack_response() || (time_millis() - config_request_time) > 500) {
      gps_status.state = state.gps_lock ? GPS_RUNNING : GPS_WAITING_FOR_LOCK;
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
  // NAV-SAT is optional diagnostics; only fresh NAV-PVT solutions keep the
  // shared fix valid. Retain the last measurements, but make the OSD blink
  // and consumers reject the fix if the receiver stops delivering solutions.
  if (time_millis() - state.gps_last_update_ms > GPS_SOLUTION_STALE_MS)
    state.gps_lock = false;
  if (time_millis() - jamming_update_ms > GPS_INTERFERENCE_STALE_MS)
    state.gps_jamming = GPS_JAMMING_UNKNOWN;
  if (time_millis() - spoofing_update_ms > GPS_INTERFERENCE_STALE_MS)
    state.gps_spoofing = GPS_SPOOFING_UNKNOWN;

  // Steady state is fed by serial RX data; the periodic backstop covers
  // config-state retries and a missed notification. The RX ring at 115200
  // baud overflows in ~44 ms, so 20 ms leaves ample margin.
  if (gps_status.state == GPS_RUNNING || gps_status.state == GPS_RUNNING_NAV_SAT_OFF)
    return pdMS_TO_TICKS(20);
  return pdMS_TO_TICKS(50);
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
TickType_t gps_task(void) { return portMAX_DELAY; }
#endif
