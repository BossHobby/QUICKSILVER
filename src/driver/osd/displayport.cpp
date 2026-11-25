#include "driver/osd/displayport.h"

#include <string.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "io/msp.h"
#include "util/crc.h"
#include "util/ring_buffer.h"
#include "util/util.h"

#ifdef USE_DIGITAL_VTX

#define MAX_MSP_FRAME_SIZE 1024
#define MAX_STRING_LENGTH 30U
// Kept free for replies to the goggles, which are dropped when they do not fit.
#define REPLY_HEADROOM 128U

typedef enum {
  SUBCMD_HEARTBEAT = 0,
  SUBCMD_RELEASE = 1,
  SUBCMD_CLEAR_SCREEN = 2,
  SUBCMD_WRITE_STRING = 3,
  SUBCMD_DRAW_SCREEN = 4,
  SUBCMD_SET_OPTIONS = 5,
} displayport_subcmd_t;

typedef enum {
  ATTR_NONE = 0,
  ATTR_INFO,
  ATTR_WARNING,
  ATTR_CRITICAL,
  ATTR_BLINK = 0x80,
} displayport_attr_t;

static DMA_RAM uint8_t tx_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = sizeof(tx_data),
};

static DMA_RAM uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = sizeof(rx_data),
};

serial_port_t serial_displayport = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

// MSP_DP_OPTIONS resolution ordinals, hardcoded by HDZero and WTFOS.
typedef enum {
  OPTIONS_SD_3016 = 0,
  OPTIONS_HD_5018 = 1,
  OPTIONS_HD_6022 = 3,
  OPTIONS_HD_5320 = 4,
} displayport_options_mode_t;

// Owned by the OSD thread.
displayport_status_t displayport_status = {
    .connected = 0,
    .cols = DISPLAYPORT_COLS,
    .rows = DISPLAYPORT_ROWS,
};
static uint8_t msp_options[2] = {0, OPTIONS_HD_5320};

// Frames are assembled only by the OSD task, which owns this port.
static uint8_t msp_frame[MAX_MSP_FRAME_SIZE];

static void displayport_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  if (cmd == MSP_FC_VARIANT) {
    // we got the first MSP_FC_VARIANT request, consider vtx detected
    displayport_status.connected = 1;
  }

  if (len > MAX_MSP_FRAME_SIZE - MSP2_HEADER_LEN - 1) {
    return; // Payload too large
  }

  if (magic == MSP2_MAGIC) {
    if (serial_bytes_free(&serial_displayport) < static_cast<uint32_t>(len + MSP2_HEADER_LEN + 1)) {
      return;
    }

    uint32_t size = 0;

    msp_frame[size++] = '$';
    msp_frame[size++] = MSP2_MAGIC;
    msp_frame[size++] = direction;

    uint8_t crc = 0;

    msp_frame[size++] = 0; // flag
    crc = crc8_dvb_s2_calc(crc, 0);

    msp_frame[size++] = (cmd >> 0) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (cmd >> 0) & 0xFF);
    msp_frame[size++] = (cmd >> 8) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (cmd >> 8) & 0xFF);
    msp_frame[size++] = (len >> 0) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (len >> 0) & 0xFF);
    msp_frame[size++] = (len >> 8) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (len >> 8) & 0xFF);

    memcpy(msp_frame + size, data, len);
    size += len;

    msp_frame[size++] = crc8_dvb_s2_data(crc, data, len);

    serial_write_bytes(&serial_displayport, msp_frame, size);
  } else if (len < 255) {
    if (serial_bytes_free(&serial_displayport) < static_cast<uint32_t>(len + MSP_HEADER_LEN + 1)) {
      return;
    }

    uint32_t size = 0;

    msp_frame[size++] = '$';
    msp_frame[size++] = MSP1_MAGIC;
    msp_frame[size++] = direction;
    msp_frame[size++] = len;
    msp_frame[size++] = cmd;

    memcpy(msp_frame + size, data, len);
    size += len;

    uint8_t chksum = len ^ cmd;
    for (uint8_t i = 0; i < len; i++) {
      chksum ^= data[i];
    }
    msp_frame[size++] = chksum;

    serial_write_bytes(&serial_displayport, msp_frame, size);
  }
}

static uint8_t msp_buffer[128];
msp_t displayport_msp = {
    .buffer = msp_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = displayport_msp_send,
    .device = MSP_DEVICE_VTX,
};


static bool displayport_push_subcmd(displayport_subcmd_t subcmd, const uint8_t *data, const uint8_t len) {
  if (len > MAX_MSP_FRAME_SIZE - MSP_HEADER_LEN - 2) {
    return false; // Data too large
  }

  if (serial_bytes_free(&serial_displayport) < static_cast<uint32_t>(MSP_HEADER_LEN + len + 2)) {
    return false;
  }

  uint32_t size = 0;

  msp_frame[size++] = '$';
  msp_frame[size++] = 'M';
  msp_frame[size++] = '>';
  msp_frame[size++] = len + 1;
  msp_frame[size++] = MSP_DISPLAYPORT;
  msp_frame[size++] = subcmd;

  memcpy(msp_frame + size, data, len);
  size += len;

  uint8_t chksum = (len + 1) ^ MSP_DISPLAYPORT ^ subcmd;
  for (uint8_t i = 0; i < len; i++) {
    chksum ^= data[i];
  }
  msp_frame[size++] = chksum;

  return serial_write_bytes(&serial_displayport, msp_frame, size);
}

static uint8_t displayport_map_attr(uint8_t attr) {
  uint8_t val = 0;
  if (attr & OSD_ATTR_INVERT) {
    val |= 0x1;
  }

  /*
  if (attr & OSD_ATTR_BLINK) {
    val |= ATTR_BLINK;
  }
  */
  return val;
}

void displayport_init() {
  serial_port_config_t config = {};
  config.port = profile.serial.digital_vtx;
  config.baudrate = 115200;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = false;
  config.half_duplex_pp = false;
  config.dma_mode = SERIAL_DMA_BOTH;

  serial_init(&serial_displayport, config);
}

static void displayport_wait_for_ready() {
  const uint32_t start = time_millis();
  while (!displayport_is_ready()) {
    if ((time_millis() - start) > 500) {
      return;
    }
  }
}

void displayport_intro() {
  displayport_wait_for_ready();
  while (!displayport_clear_async())
    ;

  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }
    displayport_wait_for_ready();
    while (!displayport_push_string(OSD_ATTR_TEXT, (displayport_status.cols / 2) - 12, (displayport_status.rows / 2) - 2 + row, buffer, 24))
      ;
  }

  while (displayport_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0))
    ;
}

bool displayport_clear_async() {
  displayport_push_subcmd(SUBCMD_SET_OPTIONS, msp_options, 2);
  return displayport_push_subcmd(SUBCMD_CLEAR_SCREEN, NULL, 0);
}

bool displayport_is_ready(TickType_t *wait) {
  bool had_packet = false;
  // Consume only the bytes already buffered at entry, even if more arrive.
  const uint32_t available = serial_bytes_available(&serial_displayport);
  uint8_t data = 0;
  for (uint32_t i = 0; i < available && serial_read_byte(&serial_displayport, &data); i++) {
    if (msp_process_serial(&displayport_msp, data) == MSP_SUCCESS)
      had_packet = true;
  }

  static bool was_detected = false;
  if (!was_detected && displayport_status.connected) {
    displayport_push_subcmd(SUBCMD_SET_OPTIONS, msp_options, 2);
    was_detected = displayport_status.connected;
    return false;
  }

  static uint32_t last_heartbeat = 0;
  if ((time_millis() - last_heartbeat) > 500)
    if (displayport_push_subcmd(SUBCMD_HEARTBEAT, NULL, 0))
      last_heartbeat = time_millis();

  if (wait) {
    const uint32_t remaining = 501 - MIN(time_millis() - last_heartbeat, 501U);
    *wait = MIN(*wait, pdMS_TO_TICKS(remaining));
  }

  return !had_packet;
}

// Called from the OSD thread while it parses goggle MSP.
void displayport_set_canvas(uint8_t cols, uint8_t rows) {
  // Menus are laid out for the SD grid, so never go below it. Analog OSDs
  // speaking DisplayPort announce that grid, 30x16 or 30x13.
  cols = constrain(cols, SD_COLS, HD_COLS_MAX);
  rows = constrain(rows, SD_ROWS_NTSC, HD_ROWS_MAX);
  displayport_status.cols = cols;
  displayport_status.rows = rows;
  // not every goggle polls MSP_FC_VARIANT first
  displayport_status.connected = 1;

  if (cols == SD_COLS) {
    msp_options[1] = OPTIONS_SD_3016;
  } else if (cols == 50 && rows == 18) {
    msp_options[1] = OPTIONS_HD_5018;
  } else if (cols == 60 && rows == 22) {
    msp_options[1] = OPTIONS_HD_6022;
  } else {
    msp_options[1] = OPTIONS_HD_5320;
  }
}

bool displayport_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  if (size > UINT8_MAX - 3) {
    return false; // String too large
  }

  uint8_t buffer[UINT8_MAX];
  buffer[0] = y;
  buffer[1] = x;
  buffer[2] = displayport_map_attr(attr);

  memcpy(buffer + 3, data, size);

  return displayport_push_subcmd(SUBCMD_WRITE_STRING, buffer, size + 3);
}

uint32_t displayport_can_fit() {
  const uint32_t free = serial_bytes_free(&serial_displayport);
  const uint32_t overhead = MSP_HEADER_LEN + 2 + 3 + 1;
  // Betaflight never sends more than 30 characters per string and DJI O4
  // drops the rest, so longer merged rows lose their right-hand elements.
  // Wait for room for a whole string: a fragment costs a full header.
  return free >= REPLY_HEADROOM + overhead + MAX_STRING_LENGTH ? MAX_STRING_LENGTH : 0;
}

bool displayport_flush() {
  return displayport_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
}

#endif
