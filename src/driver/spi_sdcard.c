#include "driver/spi_sdcard.h"

#include <string.h>

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#ifdef USE_SDCARD

typedef enum {
  SDCARD_POWER_UP,
  SDCARD_RESET,

  SDCARD_DETECT_INTERFACE,
  SDCARD_DETECT_INIT,
  SDCARD_DETECT_READ_INFO,
  SDCARD_DETECT_FINISH,

  SDCARD_DETECT_FAILED,

  SDCARD_READY,

  SDCARD_READ_MULTIPLE_START,
  SDCARD_READ_MULTIPLE_CONTINUE,
  SDCARD_READ_MULTIPLE_FINISH,
  SDCARD_READ_MULTIPLE_DONE,

  SDCARD_WRITE_MULTIPLE_START,
  SDCARD_WRITE_MULTIPLE_READY,
  SDCARD_WRITE_MULTIPLE_CONTINUE,
  SDCARD_WRITE_MULTIPLE_VERIFY,
  SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS,
  SDCARD_WRITE_MULTIPLE_FINISH,
  SDCARD_WRITE_MULTIPLE_FINISH_WAIT,
  SDCARD_WRITE_MULTIPLE_DONE

} sdcard_state_t;

typedef struct {
  uint8_t done;

  uint8_t *buf;
  uint32_t sector;
  uint32_t count;
  uint32_t count_done;
} sdcard_operation_t;

// how many cycles to delay for write confirm
#define IDLE_BYTES 16

#define SPI_SPEED_SLOW MHZ_TO_HZ(0.5)
#define SPI_SPEED_FAST MHZ_TO_HZ(25)

sdcard_info_t sdcard_info;

static volatile sdcard_state_t state = SDCARD_POWER_UP;
static sdcard_operation_t operation;

static spi_bus_device_t bus = {
    .auto_continue = false,
};

void sdcard_init() {
  if (target.sdcard_detect.pin != PIN_NONE) {
    gpio_config_t gpio_init;
    gpio_init.mode = GPIO_INPUT;
    gpio_init.drive = GPIO_DRIVE_NORMAL;
    gpio_init.output = GPIO_PUSHPULL;
    gpio_init.pull = GPIO_NO_PULL;
    gpio_pin_init(target.sdcard_detect.pin, gpio_init);
  }

  if (!target_spi_device_valid(&target.sdcard)) {
    return;
  }

  bus.port = target.flash.port;
  bus.nss = target.flash.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, SPI_SPEED_SLOW);
}

static bool sdcard_read_detect() {
  if (target.sdcard_detect.pin == PIN_NONE) {
    return true;
  }
  if (target.sdcard_detect.invert) {
    return !gpio_pin_read(target.sdcard_detect.pin);
  }
  return gpio_pin_read(target.sdcard_detect.pin);
}

static uint8_t sdcard_wait_non_idle() {
  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, NULL, 1),
  };

  for (uint16_t timeout = 8;; timeout--) {
    if (timeout == 0) {
      return 0xFF;
    }

    spi_seg_submit_wait(&bus, segs);

    if (ret != 0xFF) {
      return ret;
    }
  }
  return 0xFF;
}

static bool sdcard_wait_for_idle() {
  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(&ret, NULL, 1),
  };

  for (uint16_t timeout = 8;; timeout--) {
    if (timeout == 0) {
      return 0;
    }

    spi_seg_submit_wait(&bus, segs);

    if (ret == 0xFF) {
      return true;
    }
  }
  return false;
}

static uint8_t sdcard_command(const uint8_t cmd, const uint32_t args) {
  if (cmd != SDCARD_GO_IDLE && cmd != SDCARD_STOP_TRANSMISSION && !sdcard_wait_for_idle()) {
    return 0xFF;
  }

  uint32_t count = 0;
  spi_txn_segment_t segs[7] = {};

  segs[count++] = spi_make_seg_const(0x40 | cmd);
  segs[count++] = spi_make_seg_const(args >> 24);
  segs[count++] = spi_make_seg_const(args >> 16);
  segs[count++] = spi_make_seg_const(args >> 8);
  segs[count++] = spi_make_seg_const(args >> 0);

  // we have to send CRC while we are still in SD Bus mode
  switch (cmd) {
  case SDCARD_GO_IDLE:
    segs[count++] = spi_make_seg_const(0x95);
    break;
  case SDCARD_IF_COND:
    segs[count++] = spi_make_seg_const(0x87);
    break;
  default:
    segs[count++] = spi_make_seg_const(1);
    break;
  }

  if (cmd == SDCARD_STOP_TRANSMISSION) {
    segs[count++] = spi_make_seg_const(0xFF);
  }

  spi_seg_submit_wait_ex(&bus, segs, count);

  return sdcard_wait_non_idle();
}

static uint8_t sdcard_app_command(const uint8_t cmd, const uint32_t args) {
  sdcard_command(SDCARD_APP_CMD, 0);
  return sdcard_command(cmd, args);
}

uint32_t sdcard_read_response() {
  uint8_t buf[4];

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buf, NULL, 4),
  };
  spi_seg_submit_wait(&bus, segs);

  return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | (buf[3] << 0);
}

void sdcard_read_data(uint8_t *buf, const uint32_t size) {
  // wait for data token
  uint8_t token = sdcard_wait_non_idle();
  if (token != 0xFE) {
    return;
  }

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buf, NULL, size),
      // two bytes CRC
      spi_make_seg_const(0xFF),
      spi_make_seg_const(0xFF),
  };
  spi_seg_submit_wait(&bus, segs);
}

void sdcard_write_data(const uint8_t token, const uint8_t *buf, const uint32_t size) {
  const spi_txn_segment_t segs[] = {
      // start block
      spi_make_seg_const(token),

      spi_make_seg_buffer(NULL, (uint8_t *)buf, size),

      // two bytes CRC
      spi_make_seg_const(0xFF),
      spi_make_seg_const(0xFF),

      // write response
      spi_make_seg_const(0xFF),
  };
  spi_seg_submit_wait(&bus, segs);
}

static void sdcard_parse_csd(sdcard_csd_t *csd, uint8_t *c) {
  csd->CSD_STRUCTURE_VER = c[0] >> 6;

  if (csd->CSD_STRUCTURE_VER == 0) {
    csd->v1.TAAC = c[1];
    csd->v1.NSAC = c[2];
    csd->v1.TRAN_SPEED = c[3];
    csd->v1.CCC = (c[4] << 4) | ((c[5] & 0xF0) >> 4);
    csd->v1.READ_BL_LEN = (c[5] & 0x0F);
    csd->v1.READ_BL_PARTIAL = (c[6] & (1 << 7)) >> 7;
    csd->v1.WRITE_BLK_MISALIGN = (c[6] & (1 << 6)) >> 6;
    csd->v1.READ_BLK_MISALIGN = (c[6] & (1 << 5)) >> 5;
    csd->v1.DSR_IMP = (c[6] & (1 << 4)) >> 4;
    csd->v1.C_SIZE = ((c[6] & 0x03) << 10) | (c[7] << 2) | (c[8] >> 6);
    csd->v1.VDD_R_CURR_MIN = (c[8] & 0x38) >> 3;
    csd->v1.VDD_R_CURR_MAX = (c[8] & 0x07);
    csd->v1.VDD_W_CURR_MIN = (c[9] & 0xE0) >> 5;
    csd->v1.VDD_W_CURR_MAX = (c[9] & 0x1C) >> 2;
    csd->v1.C_SIZE_MULT = ((c[9] & 0x03) << 1) | (c[10] >> 7);
    csd->v1.ERASE_BLK_EN = (c[10] & (1 << 6)) >> 6;
    csd->v1.SECTOR_SIZE = ((c[10] & 0x3F) << 1) | (c[11] >> 7);
    csd->v1.WP_GRP_SIZE = (c[11] & 0x7F);
    csd->v1.WP_GRP_ENABLE = c[12] >> 7;
    csd->v1.R2W_FACTOR = (c[12] & 0x1C) >> 2;
    csd->v1.WRITE_BL_LEN = (c[12] & 0x03) << 2 | (c[13] >> 6);
    csd->v1.WRITE_BL_PARTIAL = (c[13] & (1 << 5)) >> 5;
    csd->v1.FILE_FORMAT_GRP = (c[14] & (1 << 7)) >> 7;
    csd->v1.COPY = (c[14] & (1 << 6)) >> 6;
    csd->v1.PERM_WRITE_PROTECT = (c[14] & (1 << 5)) >> 5;
    csd->v1.TMP_WRITE_PROTECT = (c[14] & (1 << 4)) >> 4;
    csd->v1.FILE_FORMAT = (c[14] & 0x0C) >> 2;
    csd->v1.CSD_CRC = c[15];
  } else if (csd->CSD_STRUCTURE_VER == 1) {
    csd->v2.TAAC = c[1];
    csd->v2.NSAC = c[2];
    csd->v2.TRAN_SPEED = c[3];
    csd->v2.CCC = (c[4] << 4) | ((c[5] & 0xF0) >> 4);
    csd->v2.READ_BL_LEN = (c[5] & 0x0F);
    csd->v2.READ_BL_PARTIAL = (c[6] & (1 << 7)) >> 7;
    csd->v2.WRITE_BLK_MISALIGN = (c[6] & (1 << 6)) >> 6;
    csd->v2.READ_BLK_MISALIGN = (c[6] & (1 << 5)) >> 5;
    csd->v2.DSR_IMP = (c[6] & (1 << 4)) >> 4;
    csd->v2.C_SIZE = (((uint32_t)c[7] & 0x3F) << 16) | (c[8] << 8) | c[9];
    csd->v2.ERASE_BLK_EN = (c[10] & (1 << 6)) >> 6;
    csd->v2.SECTOR_SIZE = (c[10] & 0x3F) << 1 | (c[11] >> 7);
    csd->v2.WP_GRP_SIZE = (c[11] & 0x7F);
    csd->v2.WP_GRP_ENABLE = (c[12] & (1 << 7)) >> 7;
    csd->v2.R2W_FACTOR = (c[12] & 0x1C) >> 2;
    csd->v2.WRITE_BL_LEN = ((c[12] & 0x03) << 2) | (c[13] >> 6);
    csd->v2.WRITE_BL_PARTIAL = (c[13] & (1 << 5)) >> 5;
    csd->v2.FILE_FORMAT_GRP = (c[14] & (1 << 7)) >> 7;
    csd->v2.COPY = (c[14] & (1 << 6)) >> 6;
    csd->v2.PERM_WRITE_PROTECT = (c[14] & (1 << 5)) >> 5;
    csd->v2.TMP_WRITE_PROTECT = (c[14] & (1 << 4)) >> 4;
    csd->v2.FILE_FORMAT = (c[14] & 0x0C) >> 2;
    csd->v2.CSD_CRC = c[15];
  }
}

sdcard_status_t sdcard_update() {
  if (!sdcard_read_detect()) {
    return SDCARD_WAIT;
  }

  static uint32_t delay_loops = 0;
  if (delay_loops > 0) {
    delay_loops--;
    return SDCARD_WAIT;
  }

  if (!spi_txn_ready(&bus)) {
    return SDCARD_WAIT;
  }

  switch (state) {
  case SDCARD_POWER_UP: {
    static uint32_t tries = 0;
    if (tries == 10) {
      state = SDCARD_DETECT_FAILED;
      break;
    }

    spi_txn_segment_t segs[20] = {};
    for (uint32_t i = 0; i < 20; i++) {
      segs[i] = spi_make_seg_const(0xff);
    }
    spi_seg_submit_continue(&bus, NULL, segs);

    state = SDCARD_RESET;
    delay_loops = 100;
    tries++;
    break;
  }

  case SDCARD_RESET: {
    static uint32_t tries = 0;

    uint8_t ret = sdcard_command(SDCARD_GO_IDLE, 0);
    if (ret == 0x01) {
      state = SDCARD_DETECT_INTERFACE;
      tries = 0;
    } else {
      tries++;
    }
    if (tries == 100) {
      tries = 0;
      delay_loops = 100;
      state = SDCARD_POWER_UP;
    }
    break;
  }
  case SDCARD_DETECT_INTERFACE: {
    uint8_t ret = sdcard_command(SDCARD_IF_COND, 0x1AA);
    if (ret == SDCARD_R1_IDLE) {
      uint32_t voltage_check = sdcard_read_response();
      if (voltage_check == 0x1AA) {
        // voltage check passed, got version 2
        sdcard_info.version = 2;
        state = SDCARD_DETECT_INIT;
      } else {
        state = SDCARD_DETECT_FAILED;
      }
    } else if (ret == (SDCARD_R1_ILLEGAL_COMMAND | SDCARD_R1_IDLE)) {
      // did respond with correct error, must be v1
      sdcard_info.version = 1;
      state = SDCARD_DETECT_INIT;
    } else {
      // ???
      state = SDCARD_DETECT_FAILED;
    }

    break;
  }
  case SDCARD_DETECT_INIT: {
    uint8_t ret = sdcard_app_command(SDCARD_ACMD_OD_COND, sdcard_info.version == 2 ? (1 << 30) : 0);
    if (ret == 0x0) {
      state = SDCARD_DETECT_READ_INFO;
    }
    break;
  }

  case SDCARD_DETECT_READ_INFO: {
    uint8_t ret = sdcard_command(SDCARD_OCR, 0);
    if (ret != 0x0) {
      state = SDCARD_DETECT_FAILED;
      break;
    }
    sdcard_info.ocr = sdcard_read_response();
    sdcard_info.high_capacity = (sdcard_info.ocr & (1 << 30)) != 0;

    ret = sdcard_command(SDCARD_CID, 0);
    if (ret != 0x0) {
      state = SDCARD_DETECT_FAILED;
      break;
    }
    sdcard_read_data((uint8_t *)&sdcard_info.cid, 16);

    ret = sdcard_command(SDCARD_CSD, 0);
    if (ret != 0x0) {
      state = SDCARD_DETECT_FAILED;
      break;
    }
    uint8_t csd_buffer[sizeof(sdcard_cid_t)];
    sdcard_read_data(csd_buffer, sizeof(sdcard_cid_t));
    sdcard_parse_csd(&sdcard_info.csd, csd_buffer);

    state = SDCARD_DETECT_FINISH;
    break;
  }

  case SDCARD_DETECT_FINISH: {
    if (!sdcard_info.high_capacity) {
      uint8_t ret = sdcard_command(SDACARD_SET_BLOCK_LEN, SDCARD_PAGE_SIZE);
      if (ret != 0x0) {
        state = SDCARD_DETECT_FAILED;
        break;
      }
    }

    spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, SPI_SPEED_FAST);
    state = SDCARD_READY;
    break;
  }

  case SDCARD_READ_MULTIPLE_START: {
    uint8_t token = sdcard_wait_non_idle();
    if (token == 0xFE) {
      state = SDCARD_READ_MULTIPLE_CONTINUE;

      const spi_txn_segment_t segs[] = {
          spi_make_seg_buffer(operation.buf + operation.count_done * SDCARD_PAGE_SIZE, NULL, SDCARD_PAGE_SIZE),

          // two bytes CRC
          spi_make_seg_const(0xFF),
          spi_make_seg_const(0xFF),
      };
      spi_seg_submit_continue(&bus, NULL, segs);
    }
    break;
  }

  case SDCARD_READ_MULTIPLE_CONTINUE: {
    if (!spi_txn_ready(&bus)) {
      break;
    }

    operation.count_done++;

    if (operation.count_done != operation.count) {
      state = SDCARD_READ_MULTIPLE_START;
    } else {
      state = SDCARD_READ_MULTIPLE_FINISH;
    }

    break;
  }

  case SDCARD_READ_MULTIPLE_FINISH: {
    if (sdcard_command(SDCARD_STOP_TRANSMISSION, 0) == 0x0) {
      state = SDCARD_READ_MULTIPLE_DONE;
    }
    break;
  }

  case SDCARD_WRITE_MULTIPLE_START: {
    if (!sdcard_wait_for_idle()) {
      break;
    }

    const uint32_t addr = sdcard_info.high_capacity ? operation.sector : operation.sector * SDCARD_PAGE_SIZE;
    if (sdcard_command(SDCARD_WRITE_MULTIPLE_BLOCK, addr) != 0x0) {
      break;
    }

    state = SDCARD_WRITE_MULTIPLE_READY;
    break;
  }

  case SDCARD_WRITE_MULTIPLE_CONTINUE: {
    if (!spi_txn_ready(&bus)) {
      break;
    }
    if (operation.count == operation.count_done) {
      break;
    }

    const spi_txn_segment_t segs[] = {
        // token
        spi_make_seg_const(0xFC),

        spi_make_seg_buffer(NULL, operation.buf, SDCARD_PAGE_SIZE),

        // two bytes CRC
        spi_make_seg_const(0xFF),
        spi_make_seg_const(0xFF),

        // write response
        spi_make_seg_const(0xFF),
    };
    spi_seg_submit_continue(&bus, NULL, segs);

    state = SDCARD_WRITE_MULTIPLE_VERIFY;
    break;
  }

  case SDCARD_WRITE_MULTIPLE_VERIFY: {
    if (!sdcard_wait_for_idle()) {
      break;
    }

    operation.count_done++;
    state = SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS;
    break;
  }

  case SDCARD_WRITE_MULTIPLE_FINISH: {
    const spi_txn_segment_t segs[] = {
        spi_make_seg_const(0xFD),
    };
    spi_seg_submit_continue(&bus, NULL, segs);

    state = SDCARD_WRITE_MULTIPLE_FINISH_WAIT;
    break;
  }

  case SDCARD_WRITE_MULTIPLE_FINISH_WAIT: {
    if (!sdcard_wait_for_idle()) {
      break;
    }

    state = SDCARD_WRITE_MULTIPLE_DONE;
    break;
  }

  case SDCARD_READY:
  case SDCARD_WRITE_MULTIPLE_READY:
  case SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS:
  case SDCARD_WRITE_MULTIPLE_DONE:
  case SDCARD_READ_MULTIPLE_DONE:
    return SDCARD_IDLE;

  case SDCARD_DETECT_FAILED:
    return SDCARD_ERROR;
  }

  return SDCARD_WAIT;
}

uint8_t sdcard_read_pages(uint8_t *buf, uint32_t sector, uint32_t count) {
  if (state != SDCARD_READY) {
    if (state == SDCARD_READ_MULTIPLE_DONE) {
      state = SDCARD_READY;
      return 1;
    }
    return 0;
  }

  const uint32_t addr = sdcard_info.high_capacity ? sector : sector * SDCARD_PAGE_SIZE;

  uint8_t ret = sdcard_command(SDCARD_READ_MULTIPLE_BLOCK, addr);
  if (ret != 0x0) {
    return 0;
  }

  operation.done = 0;
  operation.buf = buf;
  operation.sector = sector;
  operation.count = count;
  operation.count_done = 0;

  state = SDCARD_READ_MULTIPLE_START;
  return 0;
}

void sdcard_get_bounds(blackbox_device_bounds_t *blackbox_bounds) {
  uint64_t size = 0;
  if (sdcard_info.csd.CSD_STRUCTURE_VER == 0) {
    uint32_t block_len = (1 << sdcard_info.csd.v1.READ_BL_LEN);
    uint32_t mult = 1 << (sdcard_info.csd.v1.C_SIZE_MULT + 2);
    uint32_t blocknr = (sdcard_info.csd.v1.C_SIZE + 1) * mult;

    size = blocknr * block_len;
  } else {
    size = (sdcard_info.csd.v2.C_SIZE + 1) * (((uint64_t)512) << 10);
  }

  blackbox_bounds->page_size = SDCARD_PAGE_SIZE;
  blackbox_bounds->pages_per_sector = 1;

  blackbox_bounds->sectors = size / SDCARD_PAGE_SIZE;

  blackbox_bounds->sector_size = SDCARD_PAGE_SIZE;
  blackbox_bounds->total_size = size;
}

uint8_t sdcard_write_pages_start(uint32_t sector, uint32_t count) {
  if (state != SDCARD_READY) {
    if (state == SDCARD_WRITE_MULTIPLE_READY) {
      return 1;
    }
    return 0;
  }

  if (!sdcard_wait_for_idle()) {
    return 0;
  }

  if (sdcard_app_command(SDCARD_ACMD_SET_WR_BLK_ERASE_COUNT, count * 2) != 0x0) {
    return 0;
  }

  operation.done = 0;
  operation.buf = NULL;
  operation.sector = sector;
  operation.count = 0;
  operation.count_done = 0;

  state = SDCARD_WRITE_MULTIPLE_START;
  return 0;
}

uint8_t sdcard_write_pages_continue(uint8_t *buf) {
  if (state == SDCARD_WRITE_MULTIPLE_READY) {
    operation.buf = buf;
    operation.count++;

    state = SDCARD_WRITE_MULTIPLE_CONTINUE;
    return 0;
  }

  if (state == SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS) {
    state = SDCARD_WRITE_MULTIPLE_READY;
    return 1;
  }

  return 0;
}

uint8_t sdcard_write_pages_finish() {
  if (state == SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS) {
    state = SDCARD_WRITE_MULTIPLE_READY;
    return 0;
  }
  if (state == SDCARD_WRITE_MULTIPLE_READY) {
    state = SDCARD_WRITE_MULTIPLE_FINISH;
    return 0;
  }
  if (state == SDCARD_WRITE_MULTIPLE_DONE) {
    state = SDCARD_READY;
    return 1;
  }

  return 0;
}

uint8_t sdcard_write_page(uint8_t *buf, uint32_t sector) {
  if (state == SDCARD_READY) {
    sdcard_write_pages_start(sector, 1);
  }
  if (state == SDCARD_WRITE_MULTIPLE_SECTOR_SUCCESS) {
    sdcard_write_pages_continue(buf);
  }
  if (state == SDCARD_WRITE_MULTIPLE_DONE) {
    if (sdcard_write_pages_finish()) {
      return 1;
    }
  }
  if (state == SDCARD_WRITE_MULTIPLE_READY) {
    if (operation.count_done == 0) {
      sdcard_write_pages_continue(buf);
    } else {
      sdcard_write_pages_finish();
    }
  }
  return 0;
}

#endif