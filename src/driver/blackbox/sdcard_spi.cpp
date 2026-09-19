#include <string.h>

#include "core/project.h"
#include "driver/blackbox/sdcard.h"
#include "driver/spi.h"
#include "driver/time.h"

#if defined(USE_SDCARD) || defined(PIO_UNIT_TESTING)

#define BUSY_RETRY_US 1000

enum command_state_t {
  COMMAND_IDLE,
  COMMAND_RESPONSE,
  COMMAND_EXTENDED,
};

static struct {
  spi_bus_device_t bus;
  command_state_t command_state;
  uint8_t response[4];
  bool byte_pending;
  uint8_t byte;
  bool busy_retry;
  uint32_t busy_retry_at;
  struct {
    uint8_t *buffer;
    uint32_t size;
    bool read;
    bool pending;
    uint8_t response;
    uint8_t bytes[SDCARD_PAGE_SIZE];
  } data;
} sdcard;

static bool sdcard_spi_init() {
  sdcard = {};
  sdcard.bus.port = target.sdcard.port;
  sdcard.bus.nss = target.sdcard.nss;
  spi_bus_device_init(&sdcard.bus);
  spi_bus_device_reconfigure(&sdcard.bus, SPI_MODE_LEADING_EDGE, 400000);
  uint8_t clocks[20];
  memset(clocks, 0xff, sizeof(clocks));
  const spi_txn_segment_t segs[] = {spi_make_seg_buffer(nullptr, clocks, sizeof(clocks))};
  spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
  return true;
}

static void sdcard_spi_abort() {
  sdcard.command_state = COMMAND_IDLE;
  sdcard.byte_pending = false;
  sdcard.busy_retry = false;
  spi_csn_disable(&sdcard.bus);
}

static void sdcard_spi_configure() {
  spi_bus_device_reconfigure(&sdcard.bus, SPI_MODE_LEADING_EDGE, 25000000);
}

static bool sdcard_spi_ready() {
  if (spi_txn_ready(&sdcard.bus))
    return true;
  spi_txn_continue(&sdcard.bus);
  return false;
}

// RX destinations outlive DMA. Only the active command/data/busy phase owns
// this byte; a later pass consumes it after the transaction has completed.
static bool sdcard_spi_byte() {
  if (sdcard.byte_pending) {
    if (!sdcard_spi_ready())
      return false;
    sdcard.byte_pending = false;
    return true;
  }
  const spi_txn_segment_t segs[] = {spi_make_seg_buffer(&sdcard.byte, nullptr, 1)};
  sdcard.byte_pending = true;
  spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
  return false;
}

static sdcard_transfer_status_t sdcard_spi_busy() {
  if (!sdcard_spi_ready())
    return SDCARD_TRANSFER_WAIT;
  if (sdcard.busy_retry && int32_t(time_micros() - sdcard.busy_retry_at) < 0)
    return SDCARD_TRANSFER_WAIT;
  sdcard.busy_retry = false;
  if (!sdcard_spi_byte())
    return SDCARD_TRANSFER_WAIT;
  if (sdcard.byte == 0xff)
    return SDCARD_TRANSFER_DONE;
  // Capture notifications may wake Blackbox sooner than its timed wait.
  // Gate the transfer itself so they cannot accelerate card-busy polling.
  sdcard.busy_retry = true;
  sdcard.busy_retry_at = time_micros() + BUSY_RETRY_US;
  return SDCARD_TRANSFER_WAIT;
}

static sdcard_transfer_status_t sdcard_spi_command(uint8_t index, uint32_t argument, sdcard_response_t response, sdcard_response_data_t *result) {
  if (!sdcard_spi_ready())
    return SDCARD_TRANSFER_WAIT;
  if (sdcard.command_state == COMMAND_IDLE) {
    if (index != 0 && index != 12 && sdcard_spi_busy() != SDCARD_TRANSFER_DONE)
      return SDCARD_TRANSFER_WAIT;
    const uint8_t crc = index == 0 ? 0x95 : index == 8 ? 0x87
                                                       : 1;
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(0x40 | index, argument >> 24, argument >> 16, argument >> 8, argument),
        spi_make_seg_const(crc),
        spi_make_seg_const(0xff), // CMD12 stuff byte, otherwise first response byte.
    };
    // Only CMD12 has a stuff byte; do not discard the response of other commands.
    spi_seg_submit_ex(&sdcard.bus, {.segs = segs, .seg_count = index == 12 ? 3u : 2u,
                                  .done_fn = blackbox_device_notify_from_isr});
    spi_txn_continue(&sdcard.bus);
    sdcard.command_state = COMMAND_RESPONSE;
    return SDCARD_TRANSFER_WAIT;
  }
  if (sdcard.command_state == COMMAND_EXTENDED) {
    result->words[0] = (uint32_t(sdcard.response[0]) << 24) | (uint32_t(sdcard.response[1]) << 16) |
                       (uint32_t(sdcard.response[2]) << 8) | sdcard.response[3];
    sdcard.command_state = COMMAND_IDLE;
    return SDCARD_TRANSFER_DONE;
  }
  if (!sdcard_spi_byte())
    return SDCARD_TRANSFER_WAIT;
  const uint8_t r1 = sdcard.byte;
  if (r1 == 0xff)
    return SDCARD_TRANSFER_WAIT;
  sdcard.command_state = COMMAND_IDLE;
  result->status = r1;
  if (r1 == 0x05)
    return SDCARD_TRANSFER_UNSUPPORTED;
  if (r1 & ~1u)
    return SDCARD_TRANSFER_ERROR;
  if (response == SDCARD_RESPONSE_R3 || response == SDCARD_RESPONSE_R7) {
    const spi_txn_segment_t segs[] = {spi_make_seg_buffer(sdcard.response, nullptr, sizeof(sdcard.response))};
    spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
    sdcard.command_state = COMMAND_EXTENDED;
    return SDCARD_TRANSFER_WAIT;
  }
  return SDCARD_TRANSFER_DONE;
}

static void sdcard_spi_data_prepare(uint8_t *buffer, uint32_t size, bool read) {
  sdcard.data.buffer = buffer;
  sdcard.data.size = size;
  sdcard.data.read = read;
  sdcard.data.pending = false;
  sdcard.data.response = 0xff;
}

static sdcard_transfer_status_t sdcard_spi_data_poll() {
  if (!sdcard_spi_ready())
    return SDCARD_TRANSFER_WAIT;
  if (sdcard.data.pending) {
    if (!sdcard.data.read && sdcard.data.response == 0xff) {
      if (!sdcard_spi_byte())
        return SDCARD_TRANSFER_WAIT;
      sdcard.data.response = sdcard.byte;
      if (sdcard.data.response == 0xff)
        return SDCARD_TRANSFER_WAIT;
    }
    sdcard.data.pending = false;
    if (sdcard.data.read)
      memcpy(sdcard.data.buffer, sdcard.data.bytes, sdcard.data.size);
    else if ((sdcard.data.response & 0x1f) != 0x05)
      return SDCARD_TRANSFER_ERROR;
    return SDCARD_TRANSFER_DONE;
  }
  if (sdcard.data.read) {
    if (!sdcard_spi_byte())
      return SDCARD_TRANSFER_WAIT;
    const uint8_t token = sdcard.byte;
    if (token == 0xff)
      return SDCARD_TRANSFER_WAIT;
    if (token != 0xfe)
      return SDCARD_TRANSFER_ERROR;
    const spi_txn_segment_t segs[] = {
        spi_make_seg_buffer(sdcard.data.bytes, nullptr, sdcard.data.size),
        spi_make_seg_const(0xff, 0xff),
    };
    spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
  } else {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(0xfc),
        spi_make_seg_buffer(nullptr, sdcard.data.buffer, sdcard.data.size),
        spi_make_seg_const(0xff, 0xff),
        spi_make_seg_buffer(&sdcard.data.response, nullptr, 1),
    };
    spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
  }
  sdcard.data.pending = true;
  return SDCARD_TRANSFER_WAIT;
}

static void sdcard_spi_stop_write() {
  const spi_txn_segment_t segs[] = {spi_make_seg_const(0xfd)};
  spi_seg_submit_continue(&sdcard.bus, segs, .done_fn = blackbox_device_notify_from_isr);
}

const sdcard_transport_t sdcard_spi = {
    .spi = true,
    .init = sdcard_spi_init,
    .abort = sdcard_spi_abort,
    .configure = sdcard_spi_configure,
    .command = sdcard_spi_command,
    .data_prepare = sdcard_spi_data_prepare,
    .data_poll = sdcard_spi_data_poll,
    .busy = sdcard_spi_busy,
    .stop_write = sdcard_spi_stop_write,
};
#endif
