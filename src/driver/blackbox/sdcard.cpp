#include "driver/blackbox/sdcard.h"

#include "core/project.h"
#include "driver/gpio.h"
#include "driver/time.h"

#define SDCARD_INIT_TIMEOUT_US 2000000
#define SDCARD_COMMAND_TIMEOUT_US 100000
#define SDCARD_DATA_TIMEOUT_US 1000000

typedef enum {
  SDCARD_DETECT,
  SDCARD_POWER_UP,
  SDCARD_RESET,
  SDCARD_INTERFACE,
  SDCARD_APP_INIT,
  SDCARD_OP_COND,
  SDCARD_OCR,
  SDCARD_CID,
  SDCARD_RCA,
  SDCARD_CSD,
  SDCARD_CSD_DATA,
  SDCARD_SELECT,
  SDCARD_SELECT_READY,
  SDCARD_BLOCK_LEN,
  SDCARD_APP_BUS,
  SDCARD_BUS_WIDTH,
  SDCARD_CONFIGURE,
  SDCARD_READY,
  SDCARD_READ_COMMAND,
  SDCARD_READ_DATA,
  SDCARD_READ_STOP,
  SDCARD_READ_READY,
  SDCARD_READ_DONE,
  SDCARD_WRITE_APP,
  SDCARD_WRITE_PREERASE,
  SDCARD_WRITE_COMMAND,
  SDCARD_WRITE_READY,
  SDCARD_WRITE_DATA,
  SDCARD_WRITE_BUSY,
  SDCARD_WRITE_DONE,
  SDCARD_WRITE_FINISH,
  SDCARD_WRITE_FINISHED,
  SDCARD_FAILED,
} sdcard_state_t;

static struct {
  const sdcard_transport_t *transport;
  sdcard_state_t state;
  uint32_t blocks;
  uint32_t rca;
  bool version2;
  bool high_capacity;
  uint8_t csd[16];
  sdcard_response_data_t response;
  uint32_t init_started;
  uint32_t phase_started;
  uint32_t retry_at;
  bool command_pending;
  uint32_t command_started;
  struct {
    uint8_t *buffer;
    uint32_t page;
    uint32_t remaining;
    uint32_t written;
  } operation;
} sdcard = {.state = SDCARD_FAILED};

static void sdcard_fail() {
  sdcard.state = SDCARD_FAILED;
  if (sdcard.transport)
    sdcard.transport->abort();
}

static void sdcard_next(sdcard_state_t state) {
  sdcard.state = state;
  sdcard.phase_started = time_micros();
}

void sdcard_init(const sdcard_transport_t *transport) {
  sdcard = {.transport = transport, .state = SDCARD_FAILED};
  if (!transport)
    return;
  if (target.sdcard_detect.pin != PIN_NONE) {
    const gpio_config_t config = {
        .mode = GPIO_INPUT,
        .output = GPIO_PUSHPULL,
        .drive = GPIO_DRIVE_NORMAL,
        .pull = GPIO_NO_PULL,
    };
    gpio_pin_init(target.sdcard_detect.pin, config);
  }
  sdcard_next(SDCARD_DETECT);
}

void sdcard_init() {
  const sdcard_transport_t *transport = nullptr;
#ifdef USE_SDCARD
  if (target_sdcard_spi_valid())
    transport = &sdcard_spi;
#ifdef STM32H7
  else if (target_sdcard_sdio_valid())
    transport = &sdcard_sdio;
#endif
#endif
  sdcard_init(transport);
}

static sdcard_transfer_status_t sdcard_command_poll(uint8_t index, uint32_t argument, sdcard_response_t response) {
  if (!sdcard.command_pending) {
    sdcard.command_started = time_micros();
    sdcard.command_pending = true;
    sdcard.response = {};
  }
  auto result = sdcard.transport->command(index, argument, response, &sdcard.response);
  if (result == SDCARD_TRANSFER_WAIT && uint32_t(time_micros() - sdcard.command_started) > SDCARD_COMMAND_TIMEOUT_US)
    result = SDCARD_TRANSFER_ERROR; // A stalled transport is not a legacy CMD8 response.
  if (result != SDCARD_TRANSFER_WAIT)
    sdcard.command_pending = false;
  return result;
}

static bool sdcard_command(uint8_t index, uint32_t argument, sdcard_response_t response = SDCARD_RESPONSE_R1) {
  const auto result = sdcard_command_poll(index, argument, response);
  if (result != SDCARD_TRANSFER_WAIT && result != SDCARD_TRANSFER_DONE)
    sdcard_fail();
  if (result == SDCARD_TRANSFER_DONE && sdcard.transport->spi && sdcard.response.status != 0 &&
      sdcard.state != SDCARD_RESET && sdcard.state != SDCARD_APP_INIT && sdcard.state != SDCARD_OP_COND) {
    sdcard_fail();
    return false;
  }
  return result == SDCARD_TRANSFER_DONE;
}

static bool sdcard_app_command() {
  if (!sdcard_command(55, sdcard.rca))
    return false;
  if (!sdcard.transport->spi && !(sdcard.response.status & 0x20)) {
    sdcard_fail();
    return false;
  }
  return true;
}

static bool sdcard_busy() {
  if (sdcard.transport->spi) {
    const auto result = sdcard.transport->busy();
    if (result != SDCARD_TRANSFER_WAIT && result != SDCARD_TRANSFER_DONE)
      sdcard_fail();
    return result == SDCARD_TRANSFER_DONE;
  }
  // READY_FOR_DATA and CURRENT_STATE == TRAN, including after programming.
  return sdcard_command(13, sdcard.rca) && (sdcard.response.status & 0x1f00) == 0x900;
}

static bool sdcard_data() {
  const auto result = sdcard.transport->data_poll();
  if (result != SDCARD_TRANSFER_WAIT && result != SDCARD_TRANSFER_DONE)
    sdcard_fail();
  return result == SDCARD_TRANSFER_DONE;
}

static uint32_t sdcard_address() {
  return sdcard.high_capacity ? sdcard.operation.page : sdcard.operation.page * SDCARD_PAGE_SIZE;
}

static void sdcard_read_start() {
  sdcard.transport->data_prepare(sdcard.operation.buffer, SDCARD_PAGE_SIZE, true);
  sdcard_next(SDCARD_READ_COMMAND);
}

static void sdcard_decode_csd() {
  sdcard.blocks = sdcard_parse_csd(sdcard.csd);
  if (!sdcard.blocks || sdcard.high_capacity != ((sdcard.csd[0] >> 6) == 1))
    sdcard_fail();
  else
    sdcard_next(sdcard.transport->spi ? SDCARD_BLOCK_LEN : SDCARD_SELECT);
}

sdcard_status_t sdcard_update() {
  if (sdcard.state == SDCARD_FAILED)
    return SDCARD_ERROR;
  if (target.sdcard_detect.pin != PIN_NONE &&
      bool(gpio_pin_read(target.sdcard_detect.pin)) == bool(target.sdcard_detect.invert)) {
    if (sdcard.state == SDCARD_DETECT)
      return SDCARD_WAIT;
    sdcard_fail();
    return SDCARD_ERROR;
  }
  if (sdcard.state == SDCARD_DETECT) {
    if (!sdcard.transport->init()) {
      sdcard_fail();
      return SDCARD_ERROR;
    }
    // A card inserted after boot gets the full initialization timeout.
    sdcard.init_started = time_micros();
    sdcard.retry_at = sdcard.init_started;
    sdcard_next(SDCARD_POWER_UP);
    return SDCARD_WAIT;
  }
  const uint32_t now = time_micros();
  if (sdcard.state < SDCARD_READY && uint32_t(now - sdcard.init_started) > SDCARD_INIT_TIMEOUT_US) {
    sdcard_fail();
    return SDCARD_ERROR;
  }
  switch (sdcard.state) {
  case SDCARD_READY:
  case SDCARD_READ_DONE:
  case SDCARD_WRITE_READY:
  case SDCARD_WRITE_DONE:
  case SDCARD_WRITE_FINISHED:
    return SDCARD_IDLE;
  default:
    if (sdcard.state > SDCARD_READY && uint32_t(now - sdcard.phase_started) > SDCARD_DATA_TIMEOUT_US) {
      sdcard_fail();
      return SDCARD_ERROR;
    }
    break;
  }
  const bool spi = sdcard.transport->spi;
  switch (sdcard.state) {
  case SDCARD_POWER_UP:
    if (uint32_t(now - sdcard.init_started) >= 2000)
      sdcard_next(SDCARD_RESET);
    break;
  case SDCARD_RESET:
    if (sdcard_command(0, 0, SDCARD_RESPONSE_NONE)) {
      if (spi && sdcard.response.status != 1)
        sdcard_fail();
      else
        sdcard_next(SDCARD_INTERFACE);
    }
    break;
  case SDCARD_INTERFACE: {
    const auto result = sdcard_command_poll(8, 0x1aa, SDCARD_RESPONSE_R7);
    if ((!spi && result == SDCARD_TRANSFER_TIMEOUT) || (spi && result == SDCARD_TRANSFER_UNSUPPORTED)) {
      sdcard_next(SDCARD_APP_INIT);
    } else if (result == SDCARD_TRANSFER_DONE && sdcard.response.words[0] == 0x1aa) {
      sdcard.version2 = true;
      sdcard_next(SDCARD_APP_INIT);
    } else if (result != SDCARD_TRANSFER_WAIT) {
      sdcard_fail();
    }
    break;
  }
  case SDCARD_APP_INIT:
    if (int32_t(now - sdcard.retry_at) >= 0 && sdcard_app_command())
      sdcard_next(SDCARD_OP_COND);
    break;
  case SDCARD_OP_COND:
    if (!sdcard_command(41, (spi ? 0 : 0x00ff8000) | (sdcard.version2 ? 1u << 30 : 0),
                        spi ? SDCARD_RESPONSE_R1 : SDCARD_RESPONSE_R3))
      break;
    if (spi ? sdcard.response.status == 0 : (sdcard.response.words[0] & (1u << 31)) != 0) {
      sdcard.high_capacity = sdcard.version2 && (sdcard.response.words[0] & (1u << 30));
      sdcard_next(spi ? SDCARD_OCR : SDCARD_CID);
    } else {
      sdcard.retry_at = now + 1000;
      sdcard_next(SDCARD_APP_INIT);
    }
    break;
  case SDCARD_OCR:
    if (sdcard_command(58, 0, SDCARD_RESPONSE_R3)) {
      sdcard.high_capacity = sdcard.version2 && (sdcard.response.words[0] & (1u << 30));
      sdcard_next(SDCARD_CSD);
    }
    break;
  case SDCARD_CID:
    if (sdcard_command(2, 0, SDCARD_RESPONSE_R2))
      sdcard_next(SDCARD_RCA);
    break;
  case SDCARD_RCA:
    if (sdcard_command(3, 0, SDCARD_RESPONSE_R6)) {
      sdcard.rca = sdcard.response.words[0] & 0xffff0000;
      if (sdcard.rca)
        sdcard_next(SDCARD_CSD);
      else
        sdcard_fail();
    }
    break;
  case SDCARD_CSD:
    if (!sdcard_command(9, sdcard.rca, spi ? SDCARD_RESPONSE_R1 : SDCARD_RESPONSE_R2))
      break;
    if (spi) {
      sdcard.transport->data_prepare(sdcard.csd, sizeof(sdcard.csd), true);
      sdcard_next(SDCARD_CSD_DATA);
    } else {
      for (uint32_t i = 0; i < sizeof(sdcard.csd); i++)
        sdcard.csd[i] = sdcard.response.words[i / 4] >> (24 - (i % 4) * 8);
      sdcard_decode_csd();
    }
    break;
  case SDCARD_CSD_DATA:
    if (sdcard_data())
      sdcard_decode_csd();
    break;
  case SDCARD_SELECT:
    if (sdcard_command(7, sdcard.rca))
      sdcard_next(SDCARD_SELECT_READY);
    break;
  case SDCARD_SELECT_READY:
    if (sdcard_busy())
      sdcard_next(SDCARD_BLOCK_LEN);
    break;
  case SDCARD_BLOCK_LEN:
    if (sdcard.high_capacity || sdcard_command(16, SDCARD_PAGE_SIZE))
      sdcard_next(spi ? SDCARD_CONFIGURE : SDCARD_APP_BUS);
    break;
  case SDCARD_APP_BUS:
    if (sdcard_app_command())
      sdcard_next(SDCARD_BUS_WIDTH);
    break;
  case SDCARD_BUS_WIDTH:
    if (sdcard_command(6, 2))
      sdcard_next(SDCARD_CONFIGURE);
    break;
  case SDCARD_CONFIGURE:
    sdcard.transport->configure();
    sdcard_next(SDCARD_READY);
    break;
  case SDCARD_READ_COMMAND:
    if (sdcard_command(spi ? 18 : 17, sdcard_address()))
      sdcard_next(SDCARD_READ_DATA);
    break;
  case SDCARD_READ_DATA:
    if (!sdcard_data())
      break;
    sdcard.operation.page++;
    sdcard.operation.buffer += SDCARD_PAGE_SIZE;
    sdcard.operation.remaining--;
    if (!spi)
      sdcard_next(SDCARD_READ_READY);
    else if (!sdcard.operation.remaining)
      sdcard_next(SDCARD_READ_STOP);
    else {
      sdcard.transport->data_prepare(sdcard.operation.buffer, SDCARD_PAGE_SIZE, true);
      sdcard_next(SDCARD_READ_DATA);
    }
    break;
  case SDCARD_READ_STOP:
    if (sdcard_command(12, 0))
      sdcard_next(SDCARD_READ_READY);
    break;
  case SDCARD_READ_READY:
    if (sdcard_busy()) {
      if (sdcard.operation.remaining)
        sdcard_read_start();
      else
        sdcard_next(SDCARD_READ_DONE);
    }
    break;
  case SDCARD_WRITE_APP:
    if (sdcard_app_command())
      sdcard_next(SDCARD_WRITE_PREERASE);
    break;
  case SDCARD_WRITE_PREERASE:
    if (sdcard_command(23, sdcard.operation.remaining))
      sdcard_next(SDCARD_WRITE_COMMAND);
    break;
  case SDCARD_WRITE_COMMAND:
    if (sdcard_command(spi ? 25 : 24, sdcard_address()))
      sdcard_next(spi ? SDCARD_WRITE_READY : SDCARD_WRITE_DATA);
    break;
  case SDCARD_WRITE_DATA:
    if (sdcard_data())
      sdcard_next(SDCARD_WRITE_BUSY);
    break;
  case SDCARD_WRITE_BUSY:
    if (sdcard_busy()) {
      sdcard.operation.page++;
      sdcard.operation.written++;
      sdcard_next(SDCARD_WRITE_DONE);
    }
    break;
  case SDCARD_WRITE_FINISH:
    if (sdcard_busy())
      sdcard_next(SDCARD_WRITE_FINISHED);
    break;
  default:
    break;
  }
  return sdcard.state == SDCARD_FAILED ? SDCARD_ERROR : SDCARD_WAIT;
}

void sdcard_get_bounds(blackbox_device_bounds_t *bounds) {
  *bounds = {};
  if (!sdcard.transport)
    return;
  bounds->page_size = SDCARD_PAGE_SIZE;
  bounds->pages_per_sector = 1;
  bounds->sector_size = SDCARD_PAGE_SIZE;
  bounds->sectors = sdcard.blocks;
  bounds->total_size = bounds->sectors * SDCARD_PAGE_SIZE;
}

uint32_t sdcard_parse_csd(const uint8_t *csd) {
  const uint32_t version = csd[0] >> 6;
  uint64_t blocks = 0;
  if (version == 1) {
    const uint32_t size = (uint32_t(csd[7] & 0x3f) << 16) | (csd[8] << 8) | csd[9];
    blocks = (uint64_t(size) + 1) << 10;
  } else if (version == 0) {
    const uint32_t block_len = csd[5] & 0xf;
    const uint32_t size = ((csd[6] & 3) << 10) | (csd[7] << 2) | (csd[8] >> 6);
    const uint32_t mult = ((csd[9] & 3) << 1) | (csd[10] >> 7);
    if (block_len < 9 || block_len > 11)
      return 0;
    blocks = (uint64_t(size) + 1) << (mult + 2 + block_len - 9);
  }

  // The existing blackbox format uses 32-bit byte offsets.
  const uint32_t max_blocks = UINT32_MAX / SDCARD_PAGE_SIZE;
  return blocks > max_blocks ? max_blocks : blocks;
}

uint8_t sdcard_read_pages(uint8_t *buf, uint32_t page, uint32_t count) {
  if (sdcard.state == SDCARD_READ_DONE) {
    sdcard_next(SDCARD_READY);
    return 1;
  }
  if (sdcard.state != SDCARD_READY)
    return 0;
  if (!buf || !count || page >= sdcard.blocks || count > sdcard.blocks - page) {
    sdcard_fail();
    return 0;
  }
  sdcard.operation = {.buffer = buf, .page = page, .remaining = count};
  sdcard_read_start();
  return 0;
}

uint8_t sdcard_write_pages_start(uint32_t page, uint32_t count) {
  if (sdcard.state == SDCARD_WRITE_READY)
    return 1;
  if (sdcard.state != SDCARD_READY)
    return 0;
  // Count is a pre-erase hint, not a bound on the stream.
  if (!count || page >= sdcard.blocks) {
    sdcard_fail();
    return 0;
  }
  sdcard.operation = {.page = page, .remaining = count};
  sdcard_next(sdcard.transport->spi ? SDCARD_WRITE_APP : SDCARD_WRITE_READY);
  return 0;
}

uint8_t sdcard_write_pages_continue(uint8_t *buf) {
  if (sdcard.state == SDCARD_WRITE_DONE) {
    sdcard_next(SDCARD_WRITE_READY);
    return 1;
  }
  if (sdcard.state != SDCARD_WRITE_READY)
    return 0;
  if (!buf || sdcard.operation.page >= sdcard.blocks) {
    sdcard_fail();
    return 0;
  }
  sdcard.transport->data_prepare(buf, SDCARD_PAGE_SIZE, false);
  sdcard_next(sdcard.transport->spi ? SDCARD_WRITE_DATA : SDCARD_WRITE_COMMAND);
  return 0;
}

uint8_t sdcard_write_pages_finish() {
  if (sdcard.state == SDCARD_WRITE_FINISHED) {
    sdcard_next(SDCARD_READY);
    return 1;
  }
  if (sdcard.state == SDCARD_WRITE_READY || sdcard.state == SDCARD_WRITE_DONE) {
    if (sdcard.transport->spi)
      sdcard.transport->stop_write();
    sdcard_next(SDCARD_WRITE_FINISH);
  }
  return 0;
}

uint8_t sdcard_write_page(uint8_t *buf, uint32_t page) {
  if (sdcard.state == SDCARD_READY)
    sdcard_write_pages_start(page, 1);
  if (sdcard.state == SDCARD_WRITE_DONE)
    sdcard_write_pages_continue(buf);
  if (sdcard.state == SDCARD_WRITE_FINISHED)
    return sdcard_write_pages_finish();
  if (sdcard.state == SDCARD_WRITE_READY) {
    if (sdcard.operation.written == 0)
      sdcard_write_pages_continue(buf);
    else
      sdcard_write_pages_finish();
  }
  return 0;
}
