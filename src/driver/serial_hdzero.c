#include "driver/serial_hdzero.h"

#include <string.h>

#include "core/profile.h"
#include "driver/serial.h"
#include "io/msp.h"
#include "util/crc.h"
#include "util/ring_buffer.h"

#ifdef USE_DIGITAL_VTX

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

static uint8_t tx_data[512];
static ring_buffer_t tx_buffer = {
    .buffer = tx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static uint8_t rx_data[512];
static ring_buffer_t rx_buffer = {
    .buffer = rx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

serial_port_t serial_hdzero = {
    .rx_buffer = &rx_buffer,
    .tx_buffer = &tx_buffer,

    .tx_done = true,
};

static const uint8_t msp_options[2] = {0, 1};
static volatile uint32_t last_heartbeat = 0;
static bool is_detected = false;

static void hdzero_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  if (cmd == MSP_FC_VARIANT) {
    // we got the first MSP_FC_VARIANT request, consider vtx detected
    is_detected = true;
  }

  if (magic == MSP2_MAGIC) {
    if (serial_bytes_free(&serial_hdzero) < (len + MSP2_HEADER_LEN + 1)) {
      return;
    }

    uint32_t size = 0;
    uint8_t buf[len + MSP2_HEADER_LEN + 1];

    buf[size++] = '$';
    buf[size++] = MSP2_MAGIC;
    buf[size++] = direction;

    uint8_t crc = 0;

    buf[size++] = 0; // flag
    crc = crc8_dvb_s2_calc(crc, 0);

    buf[size++] = (cmd >> 0) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (cmd >> 0) & 0xFF);
    buf[size++] = (cmd >> 8) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (cmd >> 8) & 0xFF);
    buf[size++] = (len >> 0) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (len >> 0) & 0xFF);
    buf[size++] = (len >> 8) & 0xFF;
    crc = crc8_dvb_s2_calc(crc, (len >> 8) & 0xFF);

    memcpy(buf + size, data, len);
    size += len;

    buf[size++] = crc8_dvb_s2_data(crc, data, len);

    serial_write_bytes(&serial_hdzero, buf, size);
  } else if (len < 255) {
    if (serial_bytes_free(&serial_hdzero) < (len + MSP_HEADER_LEN + 1)) {
      return;
    }

    uint32_t size = 0;
    uint8_t buf[len + MSP_HEADER_LEN + 1];

    buf[size++] = '$';
    buf[size++] = MSP1_MAGIC;
    buf[size++] = direction;
    buf[size++] = len;
    buf[size++] = cmd;

    memcpy(buf + size, data, len);
    size += len;

    uint8_t chksum = len ^ cmd;
    for (uint8_t i = 0; i < len; i++) {
      chksum ^= data[i];
    }
    buf[size++] = chksum;

    serial_write_bytes(&serial_hdzero, buf, size);
  }
}

static uint8_t msp_buffer[128];
msp_t hdzero_msp = {
    .buffer = msp_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = hdzero_msp_send,
    .device = MSP_DEVICE_VTX,
};

static bool hdzero_push_subcmd(displayport_subcmd_t subcmd, const uint8_t *data, const uint8_t len) {
  if (serial_bytes_free(&serial_hdzero) < (MSP_HEADER_LEN + len + 2)) {
    return false;
  }

  uint32_t size = 0;
  uint8_t buf[MSP_HEADER_LEN + len + 2];

  buf[size++] = '$';
  buf[size++] = 'M';
  buf[size++] = '>';
  buf[size++] = len + 1;
  buf[size++] = MSP_DISPLAYPORT;
  buf[size++] = subcmd;

  memcpy(buf + size, data, len);
  size += len;

  uint8_t chksum = (len + 1) ^ MSP_DISPLAYPORT ^ subcmd;
  for (uint8_t i = 0; i < len; i++) {
    chksum ^= data[i];
  }
  buf[size++] = chksum;

  return serial_write_bytes(&serial_hdzero, buf, size);
}

static uint8_t hdzero_map_attr(uint8_t attr) {
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

void hdzero_init() {
  serial_port_config_t config;
  config.port = profile.serial.hdzero;
  config.baudrate = 115200;
  config.direction = SERIAL_DIR_TX_RX;
  config.stop_bits = SERIAL_STOP_BITS_1;
  config.invert = false;
  config.half_duplex = false;
  config.half_duplex_pp = false;

  serial_init(&serial_hdzero, config);
}

static void hdzero_wait_for_ready() {
  const uint32_t start = time_millis();
  while (!hdzero_is_ready()) {
    if ((time_millis() - start) > 500) {
      return;
    }
  }
}

void hdzero_intro() {
  hdzero_wait_for_ready();
  hdzero_clear_async();

  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }
    hdzero_wait_for_ready();
    hdzero_push_string(OSD_ATTR_TEXT, (HDZERO_COLS / 2) - 12, (HDZERO_ROWS / 2) - 2 + row, buffer, 24);
  }

  hdzero_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
}

uint8_t hdzero_clear_async() {
  hdzero_push_subcmd(SUBCMD_SET_OPTIONS, msp_options, 2);
  return hdzero_push_subcmd(SUBCMD_CLEAR_SCREEN, NULL, 0);
}

bool hdzero_is_ready() {
  while (true) {
    uint8_t data = 0;
    if (!serial_read_bytes(&serial_hdzero, &data, 1)) {
      break;
    }

    msp_process_serial(&hdzero_msp, data);
  }

  static bool was_detected = false;
  if (!was_detected && is_detected) {
    hdzero_push_subcmd(SUBCMD_SET_OPTIONS, msp_options, 2);
    was_detected = is_detected;
    return false;
  }

  if ((time_millis() - last_heartbeat) > 500) {
    hdzero_push_subcmd(SUBCMD_HEARTBEAT, NULL, 0);
    last_heartbeat = time_millis();
    return false;
  }

  return true;
}

osd_system_t hdzero_check_system() {
  return OSD_SYS_HD;
}

bool hdzero_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  uint8_t buffer[size + 3];
  buffer[0] = y;
  buffer[1] = x;
  buffer[2] = hdzero_map_attr(attr);

  memcpy(buffer + 3, data, size);

  return hdzero_push_subcmd(SUBCMD_WRITE_STRING, buffer, size + 3);
}

bool hdzero_can_fit(uint8_t size) {
  const uint32_t free = serial_bytes_free(&serial_hdzero);
  return free > (size + 10);
}

bool hdzero_flush() {
  return hdzero_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
}

#endif