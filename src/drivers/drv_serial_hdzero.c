#include "drv_serial_hdzero.h"

#include <string.h>

#include "drv_msp.h"
#include "drv_serial.h"
#include "profile.h"
#include "util/circular_buffer.h"

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

#define ROWS 18
#define COLS 50

#define BUFFER_SIZE 128

#define USART usart_port_defs[serial_hdzero_port]

static volatile uint32_t last_heartbeat = 0;
static bool is_detected = false;

static uint8_t msp_frame[BUFFER_SIZE];
static volatile uint8_t msp_frame_length = 0;
static volatile uint8_t msp_frame_offset = 0;
static volatile uint8_t msp_transfer_done = 1;

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

static void hdzero_start() {
  while (!msp_transfer_done)
    ;

  msp_frame_length = 0;
}

static void hdzero_push_msp(const uint8_t code, const uint8_t *data, const uint8_t size) {
  const uint8_t full_size = size + MSP_HEADER_LEN + 1;

  uint8_t *frame = msp_frame + msp_frame_length;
  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '>';
  frame[3] = size;
  frame[4] = code;

  for (uint8_t i = 0; i < size; i++) {
    frame[i + MSP_HEADER_LEN] = data[i];
  }

  uint8_t chksum = size;
  for (uint8_t i = 4; i < full_size; i++) {
    chksum ^= frame[i];
  }
  frame[size + MSP_HEADER_LEN] = chksum;

  msp_frame_length += full_size;
}

static void hdzero_push_subcmd(displayport_subcmd_t subcmd, const uint8_t *data, const uint8_t size) {
  const uint8_t full_size = size + MSP_HEADER_LEN + 2;

  uint8_t *frame = msp_frame + msp_frame_length;
  frame[0] = '$';
  frame[1] = 'M';
  frame[2] = '>';
  frame[3] = size + 1;
  frame[4] = MSP_DISPLAYPORT;
  frame[5] = subcmd;

  for (uint8_t i = 0; i < size; i++) {
    frame[i + MSP_HEADER_LEN + 1] = data[i];
  }

  uint8_t chksum = size + 1;
  for (uint8_t i = 4; i < (full_size - 1); i++) {
    chksum ^= frame[i];
  }
  frame[size + MSP_HEADER_LEN + 1] = chksum;

  msp_frame_length += full_size;
}

static void hdzero_submit() {
  if (msp_frame_length == 0) {
    return;
  }

  msp_transfer_done = 0;
  msp_frame_offset = 0;

  LL_USART_ClearFlag_TC(USART.channel);

  LL_USART_EnableIT_TXE(USART.channel);
  LL_USART_EnableIT_TC(USART.channel);
}

void hdzero_init() {
  serial_hdzero_port = profile.serial.hdzero;

  serial_enable_rcc(serial_hdzero_port);
  serial_init(serial_hdzero_port, 115200, false);
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

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(OSD_ATTR_TEXT, COLS / 2 - 6, ROWS / 2 - 1);
  osd_txn_write_str("QUICKSILVER");
  osd_txn_submit(txn);
}

uint8_t hdzero_clear_async() {
  hdzero_start();
  hdzero_push_subcmd(SUBCMD_CLEAR_SCREEN, NULL, 0);
  hdzero_submit();
  return 1;
}

bool hdzero_is_ready() {
  if (!msp_transfer_done) {
    return false;
  }
  if ((time_millis() - last_heartbeat) > 500) {
    return false;
  }
  static bool wants_heatbeat = true;
  if (wants_heatbeat) {
    hdzero_start();
    uint8_t variant[6] = {'A', 'R', 'D', 'U', 0, 0};
    hdzero_push_msp(MSP_FC_VARIANT, variant, 6);
    hdzero_submit();

    hdzero_start();
    uint8_t options[2] = {0, 1};
    hdzero_push_subcmd(SUBCMD_SET_OPTIONS, options, 2);
    hdzero_submit();

    wants_heatbeat = false;
    return false;
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

void hdzero_txn_submit(osd_transaction_t *txn) {
  hdzero_start();

  for (uint16_t i = 0; i < txn->segment_count; i++) {
    const osd_segment_t *seg = &txn->segments[i];
    const uint8_t size = seg->size + 3;

    uint8_t buffer[size];
    buffer[0] = seg->y;
    buffer[1] = seg->x;
    buffer[2] = hdzero_map_attr(seg->attr);

    memcpy(buffer + 3, txn->buffer + seg->offset, seg->size);

    hdzero_push_subcmd(SUBCMD_WRITE_STRING, buffer, size);
  }

  hdzero_push_subcmd(SUBCMD_DRAW_SCREEN, NULL, 0);
  hdzero_submit();
}

void hdzero_uart_isr() {
  if (LL_USART_IsEnabledIT_TC(USART.channel) && LL_USART_IsActiveFlag_TC(USART.channel)) {
    LL_USART_ClearFlag_TC(USART.channel);
    if (msp_frame_offset == msp_frame_length && msp_transfer_done == 0) {
      msp_transfer_done = 1;
    }
  }

  if (LL_USART_IsEnabledIT_TXE(USART.channel) && LL_USART_IsActiveFlag_TXE(USART.channel)) {
    if (msp_frame_offset < msp_frame_length) {
      LL_USART_TransmitData8(USART.channel, msp_frame[msp_frame_offset]);
      msp_frame_offset++;
    }
    if (msp_frame_offset == msp_frame_length) {
      LL_USART_DisableIT_TXE(USART.channel);
    }
  }

  if (LL_USART_IsEnabledIT_RXNE(USART.channel) && LL_USART_IsActiveFlag_RXNE(USART.channel)) {
    // clear the rx flag by reading, but discard the data
    volatile uint8_t data = LL_USART_ReceiveData8(USART.channel);
    data = data;

    last_heartbeat = time_millis();
  }

  if (LL_USART_IsActiveFlag_ORE(USART.channel)) {
    LL_USART_ClearFlag_ORE(USART.channel);
  }
}