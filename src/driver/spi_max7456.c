#include "driver/spi_max7456.h"

#include <stdio.h>

#include "core/project.h"
#include "driver/osd.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "string.h"
#include "util/util.h"

#ifdef USE_MAX7456

#define DMA_BUFFER_SIZE 128
#define MAX7456_BAUD_RATE MHZ_TO_HZ(10.5)

// osd video system (PAL/NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
static osd_system_t current_osd_system = OSD_SYS_NTSC;
static osd_system_t last_osd_system = OSD_SYS_NONE;

static spi_bus_device_t bus = {
    .auto_continue = true,
};

static uint8_t max7456_map_attr(uint8_t attr) {
  // we always want at least text
  uint8_t val = TEXT;

  if (attr & OSD_ATTR_INVERT) {
    val |= INVERT;
  }

  if (attr & OSD_ATTR_BLINK) {
    val |= BLINK;
  }

  return val;
}

// blocking dma read of a single register
static uint8_t max7456_dma_spi_read(uint8_t reg) {
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  uint8_t ret = 0;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_buffer(&ret, NULL, 1),
  };
  spi_seg_submit_wait(&bus, segs);

  return ret;
}

// blocking dma write of a single register
static void max7456_dma_spi_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg, data),
  };
  spi_seg_submit_wait(&bus, segs);
}

static bool max7456_init_display() {
  max7456_dma_spi_write(VM0, 0x02); // soft reset
  time_delay_us(200);

  const uint8_t stat = max7456_dma_spi_read(STAT);
  if (stat & 0x40) {
    // reset did not go low, something is wrong
    return false;
  }

  const uint8_t black_level = max7456_dma_spi_read(OSDBL_R);
  max7456_dma_spi_write(OSDBL_W, black_level | 0x10);

  switch (current_osd_system) {
  case OSD_SYS_PAL:
    max7456_dma_spi_write(VM0, 0x78); // Set pal mode and enable display
    break;

  case OSD_SYS_NTSC:
    max7456_dma_spi_write(VM0, 0x08); // Set ntsc mode and enable display
    break;

  default:
    break;
  }

  max7456_dma_spi_write(VM1, 0x0C);  // set background brightness (bits 456), blinking time(bits 23), blinking duty cycle (bits 01)
  max7456_dma_spi_write(OSDM, 0x2D); // osd mux & rise/fall ( lowest sharpness)

  last_osd_system = OSD_SYS_NONE;
  max7456_check_system();

  return true;
}

// establish initial boot-up state
bool max7456_init() {
  bus.port = target.osd.port;
  bus.nss = target.osd.nss;
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  return max7456_init_display();
}

// set the video output system PAL /NTSC
static void max7456_set_system(osd_system_t sys) {
  if (current_osd_system == sys) {
    return;
  }

  const uint8_t vm0 = max7456_dma_spi_read(VM0_R);
  switch (sys) {
  case OSD_SYS_PAL:
    max7456_dma_spi_write(VM0, vm0 | 0x40);
    break;

  case OSD_SYS_NTSC:
    max7456_dma_spi_write(VM0, vm0 & 0xBF);
    break;

  default:
    break;
  }

  current_osd_system = sys;
}

static osd_system_t max7456_current_system() {
  const uint8_t stat = max7456_dma_spi_read(STAT);

  if ((stat & 0x01) == 0x01) {
    return OSD_SYS_PAL;
  }
  if ((stat & 0x02) == 0x02) {
    return OSD_SYS_NTSC;
  }

  // really (x & 0x03) == 0x03 equals Loss-of-Sync
  // but its probably safe to assume if we havent detected
  // either NTSC or PAl we dont have a cam present
  return OSD_SYS_NONE;
}

uint8_t max7456_clear_async() {
  if (!max7456_is_ready()) {
    return 0;
  }

  static uint8_t row = 0;

  static const uint8_t buffer[] = "                                ";
  max7456_push_string(OSD_ATTR_TEXT, 0, row, buffer, 32);

  row++;

  if (row > MAX7456_ROWS) {
    row = 0;
    return 1;
  }
  return 0;
}

// function to detect and correct ntsc/pal mode or mismatch
// returns the current system
osd_system_t max7456_check_system() {
  if (!max7456_is_ready() || last_osd_system > OSD_SYS_NONE) {
    return last_osd_system;
  }

  const osd_system_t sys = max7456_current_system();

  switch (sys) {
  case OSD_SYS_PAL:
    if (last_osd_system != OSD_SYS_PAL) {
      last_osd_system = OSD_SYS_PAL;
      max7456_set_system(OSD_SYS_PAL);

      // initial screen clear off
      osd_clear();
    }
    break;

  case OSD_SYS_NTSC:
    if (last_osd_system != OSD_SYS_NTSC) {
      last_osd_system = OSD_SYS_NTSC;
      max7456_set_system(OSD_SYS_NTSC);

      // initial screen clear off
      osd_clear();
    }
    break;

  default: {
    static uint8_t warning_sent = 0;

    if (last_osd_system != OSD_SYS_NONE) {
      // we lost sync
      last_osd_system = OSD_SYS_NONE;
      warning_sent = 0;
      break;
    }

    if (warning_sent == 0) {
      // initial screen clear off on first run
      if (osd_clear_async()) {
        warning_sent++;
      }
      break;
    }

    if (warning_sent == 1) {
      osd_start(OSD_ATTR_BLINK, SYSTEMXPOS, SYSTEMYPOS);
      osd_write_str("NO CAMERA SIGNAL");
      warning_sent++;
      break;
    }

    break;
  }
  }

  return sys;
}

// splash screen
void max7456_intro() {
  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }

    max7456_push_string(OSD_ATTR_TEXT, 3, row + 5, buffer, 24);
    spi_txn_wait(&bus);
  }
}

bool max7456_can_fit(uint8_t size) {
  return max7456_is_ready();
}

bool max7456_push_string(uint8_t attr, uint8_t x, uint8_t y, const uint8_t *data, uint8_t size) {
  // NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
  if (last_osd_system != OSD_SYS_PAL && y > 12) {
    y = y - 2;
  }
  if (y > MAX7456_ROWS - 1) {
    y = MAX7456_ROWS - 1;
  }

  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  uint32_t offset = 0;
  uint8_t buf[8 + size * 2];

  buf[offset++] = DMM;
  buf[offset++] = max7456_map_attr(attr);

  const uint16_t pos = x + y * 30;
  buf[offset++] = DMAH;
  buf[offset++] = (pos >> 8) & 0xFF;
  buf[offset++] = DMAL;
  buf[offset++] = pos & 0xFF;

  for (uint8_t i = 0; i < size; i++) {
    buf[offset++] = DMDI;
    buf[offset++] = data[i];
  }

  // off autoincrement mode
  buf[offset++] = DMDI;
  buf[offset++] = 0xFF;

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(NULL, buf, offset),
  };
  spi_seg_submit_continue(&bus, NULL, segs);

  return true;
}

bool max7456_flush() {
  return true;
}

bool max7456_is_ready() {
  if (!spi_txn_ready(&bus)) {
    spi_txn_continue(&bus);
    return false;
  }
  return true;
}

void osd_read_character(uint8_t addr, uint8_t *out, const uint8_t size) {
  // make sure we do not collide with a dma write
  spi_txn_wait(&bus);

  // disable osd
  max7456_dma_spi_write(VM0, 0x0);
  time_delay_us(10);

  max7456_dma_spi_write(CMAH, addr);
  max7456_dma_spi_write(CMM, 0x50);

  // wait for NVM to be ready
  WHILE_TIMEOUT(max7456_dma_spi_read(STAT) & 0x20, 200);

  for (uint8_t i = 0; i < size; i++) {
    max7456_dma_spi_write(CMAL, i);
    out[i] = max7456_dma_spi_read(CMDO);
    time_delay_us(1);
  }

  // enable osd
  max7456_init_display();
}

void osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size) {
  // make sure we do not collide with a dma write
  spi_txn_wait(&bus);

  // disable osd
  max7456_dma_spi_write(VM0, 0x0);
  time_delay_us(10);

  max7456_dma_spi_write(CMAH, addr);
  for (uint8_t i = 0; i < size; i++) {
    max7456_dma_spi_write(CMAL, i);
    max7456_dma_spi_write(CMDI, in[i]);
    time_delay_us(1);
  }

  max7456_dma_spi_write(CMM, 0xA0);

  // wait for NVM to be ready
  WHILE_TIMEOUT(max7456_dma_spi_read(STAT) & 0x20, 200);

  // enable osd
  max7456_init_display();
}

#endif