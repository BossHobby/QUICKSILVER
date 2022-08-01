#include "drv_serial_hdzero.h"

#include <string.h>

#include "drv_serial.h"
#include "io/msp.h"
#include "profile.h"
#include "util/circular_buffer.h"
#include "util/crc.h"

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

#define USART usart_port_defs[serial_hdzero_port]

static volatile uint32_t last_heartbeat = 0;
static bool is_detected = false;

static uint8_t msp_tx_data[512];
static circular_buffer_t msp_tx_buffer = {
    .buffer = msp_tx_data,
    .head = 0,
    .tail = 0,
    .size = 512,
};

static uint8_t msp_rx_data[256];
static circular_buffer_t msp_rx_buffer = {
    .buffer = msp_rx_data,
    .head = 0,
    .tail = 0,
    .size = 256,
};

static const uint8_t variant[6] = {'B', 'T', 'F', 'L', 0, 0};

static void hdzero_msp_send(msp_magic_t magic, uint8_t direction, uint16_t cmd, const uint8_t *data, uint16_t len) {
  if (cmd == MSP_FC_VARIANT) {
    data = variant;
    len = 6;
  }

  if (magic == MSP2_MAGIC) {
    if (circular_buffer_free(&msp_tx_buffer) < (len + MSP2_HEADER_LEN + 1)) {
      return;
    }

    circular_buffer_write(&msp_tx_buffer, '$');
    circular_buffer_write(&msp_tx_buffer, MSP2_MAGIC);
    circular_buffer_write(&msp_tx_buffer, '>');

    uint8_t crc = 0;

    circular_buffer_write(&msp_tx_buffer, 0); // flag
    crc = crc8_dvb_s2_calc(crc, 0);

    circular_buffer_write(&msp_tx_buffer, (cmd >> 0) & 0xFF);
    crc = crc8_dvb_s2_calc(crc, (cmd >> 0) & 0xFF);
    circular_buffer_write(&msp_tx_buffer, (cmd >> 8) & 0xFF);
    crc = crc8_dvb_s2_calc(crc, (cmd >> 8) & 0xFF);
    circular_buffer_write(&msp_tx_buffer, (len >> 0) & 0xFF);
    crc = crc8_dvb_s2_calc(crc, (len >> 0) & 0xFF);
    circular_buffer_write(&msp_tx_buffer, (len >> 8) & 0xFF);
    crc = crc8_dvb_s2_calc(crc, (len >> 8) & 0xFF);

    circular_buffer_write_multi(&msp_tx_buffer, data, len);

    circular_buffer_write(&msp_tx_buffer, crc8_dvb_s2_data(crc, data, len));
  } else {
    if (circular_buffer_free(&msp_tx_buffer) < (len + MSP_HEADER_LEN + 1)) {
      return;
    }

    circular_buffer_write(&msp_tx_buffer, '$');
    circular_buffer_write(&msp_tx_buffer, MSP1_MAGIC);
    circular_buffer_write(&msp_tx_buffer, '>');
    circular_buffer_write(&msp_tx_buffer, len);
    circular_buffer_write(&msp_tx_buffer, cmd);

    circular_buffer_write_multi(&msp_tx_buffer, data, len);

    uint8_t chksum = len ^ cmd;
    for (uint8_t i = 0; i < len; i++) {
      chksum ^= data[i];
    }
    circular_buffer_write(&msp_tx_buffer, chksum);
  }

  LL_USART_EnableIT_TXE(USART.channel);
}

static uint8_t msp_buffer[128];
msp_t hdzero_msp = {
    .buffer = msp_buffer,
    .buffer_size = 128,
    .buffer_offset = 0,
    .send = hdzero_msp_send,
};

static void hdzero_push_msp(const uint8_t code, const uint8_t *data, const uint8_t size) {
  hdzero_msp_send(MSP1_MAGIC, '>', code, data, size);
}

static bool hdzero_push_subcmd(displayport_subcmd_t subcmd, const uint8_t *data, const uint8_t size) {
  if (circular_buffer_free(&msp_tx_buffer) < (MSP_HEADER_LEN + size + 2)) {
    return false;
  }

  circular_buffer_write(&msp_tx_buffer, '$');
  circular_buffer_write(&msp_tx_buffer, 'M');
  circular_buffer_write(&msp_tx_buffer, '>');
  circular_buffer_write(&msp_tx_buffer, size + 1);
  circular_buffer_write(&msp_tx_buffer, MSP_DISPLAYPORT);
  circular_buffer_write(&msp_tx_buffer, subcmd);

  circular_buffer_write_multi(&msp_tx_buffer, data, size);

  uint8_t chksum = (size + 1) ^ MSP_DISPLAYPORT ^ subcmd;
  for (uint8_t i = 0; i < size; i++) {
    chksum ^= data[i];
  }
  circular_buffer_write(&msp_tx_buffer, chksum);

  LL_USART_EnableIT_TXE(USART.channel);
  return true;
}

static uint8_t hdzero_map_attr(uint8_t attr) {
  uint8_t val = 0;
  /*
  if (attr & OSD_ATTR_INVERT) {
    val |= ATTR_WARNING;
  }

  if (attr & OSD_ATTR_BLINK) {
    val |= ATTR_BLINK;
  }
  */
  return val;
}

void hdzero_init() {
  serial_hdzero_port = profile.serial.hdzero;

  serial_enable_rcc(serial_hdzero_port);
  serial_init(NULL, serial_hdzero_port, 115200, 1, false);
  serial_enable_isr(serial_hdzero_port);

  LL_USART_EnableIT_RXNE(USART.channel);
}

void hdzero_intro() {
  const uint32_t start = time_millis();
  while (!hdzero_is_ready()) {
    if ((time_millis() - start) > 500) {
      return;
    }
  }

  hdzero_push_string(OSD_ATTR_TEXT, HDZERO_COLS / 2 - 6, HDZERO_ROWS / 2 - 1, (uint8_t *)"QUICKSILVER", 12);
  hdzero_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
}

uint8_t hdzero_clear_async() {
  hdzero_push_subcmd(SUBCMD_CLEAR_SCREEN, NULL, 0);
  return 1;
}

bool hdzero_is_ready() {
  static bool wants_heatbeat = true;

  if ((time_millis() - last_heartbeat) > 500) {
    wants_heatbeat = true;
    return false;
  }

  if (wants_heatbeat) {
    hdzero_push_msp(MSP_FC_VARIANT, variant, 6);

    uint8_t options[2] = {0, 1};
    hdzero_push_subcmd(SUBCMD_SET_OPTIONS, options, 2);

    hdzero_clear_async();
    wants_heatbeat = false;
    return false;
  }

  while (true) {
    uint8_t data = 0;
    if (!circular_buffer_read(&msp_rx_buffer, &data)) {
      break;
    }

    msp_process_serial(&hdzero_msp, data);
  }

  return true;
}

osd_system_t hdzero_check_system() {
  if ((time_millis() - last_heartbeat) > 250) {
    // timeout
    is_detected = false;
  } else if (!is_detected) {
    // detected now, but previously was not
    is_detected = true;
  }
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
  const uint32_t free = circular_buffer_free(&msp_tx_buffer);
  return free > (size + 10);
}

bool hdzero_flush() {
  return hdzero_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
}

void hdzero_uart_isr() {
  if (LL_USART_IsEnabledIT_TC(USART.channel) && LL_USART_IsActiveFlag_TC(USART.channel)) {
    LL_USART_ClearFlag_TC(USART.channel);
  }

  if (LL_USART_IsEnabledIT_TXE(USART.channel) && LL_USART_IsActiveFlag_TXE(USART.channel)) {
    uint8_t data = 0;
    if (circular_buffer_read(&msp_tx_buffer, &data)) {
      LL_USART_TransmitData8(USART.channel, data);
    } else {
      LL_USART_DisableIT_TXE(USART.channel);
    }
  }

  if (LL_USART_IsEnabledIT_RXNE(USART.channel) && LL_USART_IsActiveFlag_RXNE(USART.channel)) {
    // clear the rx flag by reading, but discard the data
    const uint8_t data = LL_USART_ReceiveData8(USART.channel);
    circular_buffer_write(&msp_rx_buffer, data);
    last_heartbeat = time_millis();
  }

  if (LL_USART_IsActiveFlag_ORE(USART.channel)) {
    LL_USART_ClearFlag_ORE(USART.channel);
  }
}