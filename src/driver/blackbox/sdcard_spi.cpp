#include <string.h>

#include "core/project.h"
#include "driver/blackbox/sdcard.h"
#include "driver/spi.h"

#ifdef USE_SDCARD

static struct {
  spi_bus_device_t bus;
  bool command_pending;
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
  spi_seg_submit_wait(&sdcard.bus, segs);
  return true;
}

static void sdcard_spi_abort() {
  sdcard.command_pending = false;
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

static uint8_t sdcard_spi_byte() {
  uint8_t byte = 0xff;
  const spi_txn_segment_t segs[] = {spi_make_seg_buffer(&byte, nullptr, 1)};
  spi_seg_submit_wait(&sdcard.bus, segs);
  return byte;
}

static sdcard_transfer_status_t sdcard_spi_busy() {
  if (!sdcard_spi_ready())
    return SDCARD_TRANSFER_WAIT;
  return sdcard_spi_byte() == 0xff ? SDCARD_TRANSFER_DONE : SDCARD_TRANSFER_WAIT;
}

static sdcard_transfer_status_t sdcard_spi_command(uint8_t index, uint32_t argument, sdcard_response_t response, sdcard_response_data_t *result) {
  if (!sdcard_spi_ready())
    return SDCARD_TRANSFER_WAIT;
  if (!sdcard.command_pending) {
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
    spi_seg_submit_wait_ex(&sdcard.bus, segs, index == 12 ? 3 : 2);
    sdcard.command_pending = true;
  }
  const uint8_t r1 = sdcard_spi_byte();
  if (r1 == 0xff)
    return SDCARD_TRANSFER_WAIT;
  sdcard.command_pending = false;
  result->status = r1;
  if (r1 == 0x05)
    return SDCARD_TRANSFER_UNSUPPORTED;
  if (r1 & ~1u)
    return SDCARD_TRANSFER_ERROR;
  if (response == SDCARD_RESPONSE_R3 || response == SDCARD_RESPONSE_R7) {
    uint8_t bytes[4];
    const spi_txn_segment_t segs[] = {spi_make_seg_buffer(bytes, nullptr, sizeof(bytes))};
    spi_seg_submit_wait(&sdcard.bus, segs);
    result->words[0] = (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) | (uint32_t(bytes[2]) << 8) | bytes[3];
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
      sdcard.data.response = sdcard_spi_byte();
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
    const uint8_t token = sdcard_spi_byte();
    if (token == 0xff)
      return SDCARD_TRANSFER_WAIT;
    if (token != 0xfe)
      return SDCARD_TRANSFER_ERROR;
    const spi_txn_segment_t segs[] = {
        spi_make_seg_buffer(sdcard.data.bytes, nullptr, sdcard.data.size),
        spi_make_seg_const(0xff, 0xff),
    };
    spi_seg_submit_continue(&sdcard.bus, segs);
  } else {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(0xfc),
        spi_make_seg_buffer(nullptr, sdcard.data.buffer, sdcard.data.size),
        spi_make_seg_const(0xff, 0xff),
        spi_make_seg_buffer(&sdcard.data.response, nullptr, 1),
    };
    spi_seg_submit_continue(&sdcard.bus, segs);
  }
  sdcard.data.pending = true;
  return SDCARD_TRANSFER_WAIT;
}

static void sdcard_spi_stop_write() {
  const spi_txn_segment_t segs[] = {spi_make_seg_const(0xfd)};
  spi_seg_submit_continue(&sdcard.bus, segs);
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
