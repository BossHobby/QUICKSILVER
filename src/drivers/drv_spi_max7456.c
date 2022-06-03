#include "drv_spi_max7456.h"

#include <stdio.h>

#include "drv_osd.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"
#include "string.h"
#include "util/util.h"

#ifdef ENABLE_OSD

#define DMA_BUFFER_SIZE 128
#define MAX7456_BAUD_RATE spi_find_divder(MHZ_TO_HZ(10.5))

// osd video system (PAL/NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
static osd_system_t current_osd_system = OSD_SYS_NTSC;
static osd_system_t last_osd_system = OSD_SYS_NONE;

// detected osd video system starts at 99 and gets updated here by osd_checksystem()
static uint8_t lastvm0 = 0x55;

static uint8_t dma_buffer[DMA_BUFFER_SIZE];
static uint16_t dma_offset = 0;

static DMA_RAM uint8_t buffer[DMA_BUFFER_SIZE * 2];
static spi_bus_device_t bus = {
    .port = MAX7456_SPI_PORT,
    .nss = MAX7456_NSS,

    .buffer = buffer,
    .buffer_size = DMA_BUFFER_SIZE * 2,

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

  uint8_t buffer[2] = {reg, 0xFF};

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return buffer[1];
}

// blocking dma write of a single register
static void max7456_dma_spi_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);
}

static void max7456_init_display() {
  max7456_dma_spi_write(VM0, 0x02); // soft reset
  time_delay_us(200);

  const uint8_t x = max7456_dma_spi_read(OSDBL_R);
  max7456_dma_spi_write(OSDBL_W, x | 0x10);

  switch (current_osd_system) {
  case OSD_SYS_PAL:
    max7456_dma_spi_write(VM0, 0x78); // Set pal mode and enable display
    lastvm0 = 0x78;
    break;

  case OSD_SYS_NTSC:
    max7456_dma_spi_write(VM0, 0x08); // Set ntsc mode and enable display
    lastvm0 = 0x08;
    break;

  default:
    break;
  }

  max7456_dma_spi_write(VM1, 0x0C);  // set background brightness (bits 456), blinking time(bits 23), blinking duty cycle (bits 01)
  max7456_dma_spi_write(OSDM, 0x2D); // osd mux & rise/fall ( lowest sharpness)

  last_osd_system = OSD_SYS_NONE;
  max7456_check_system();
}

// establish initial boot-up state
void max7456_init() {
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  max7456_init_display();
}

// non blocking bulk dma transmit for interrupt callback configuration
static void max7456_dma_it_transfer_bytes(const uint8_t *buffer, const uint8_t size) {
  spi_bus_device_reconfigure(&bus, SPI_MODE_LEADING_EDGE, MAX7456_BAUD_RATE);

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, NULL, buffer, size);
  spi_txn_submit(txn);

  spi_txn_continue(&bus);
}

// set the video output system PAL /NTSC
static void max7456_set_system(osd_system_t sys) {
  if (current_osd_system == sys) {
    return;
  }

  uint8_t x = max7456_dma_spi_read(VM0_R);

  switch (sys) {
  case OSD_SYS_PAL:
    lastvm0 = x | 0x40;
    max7456_dma_spi_write(VM0, x | 0x40);
    break;

  case OSD_SYS_NTSC:
    lastvm0 = x & 0xBF;
    max7456_dma_spi_write(VM0, x & 0xBF);
    break;

  default:
    break;
  }

  current_osd_system = sys;
}

static osd_system_t max7456_current_system() {
  uint8_t x = max7456_dma_spi_read(STAT);
  if ((x & 0x01) == 0x01) {
    return OSD_SYS_PAL;
  }
  if ((x & 0x02) == 0x02) {
    return OSD_SYS_NTSC;
  }

  // really (x & 0x03) == 0x03 equals Loss-of-Sync
  // but its probably safe to assume if we havent detected
  // either NTSC or PAl we dont have a cam present
  return OSD_SYS_NONE;
}

uint8_t max7456_clear_async() {
  spi_txn_wait(&bus);

  static uint8_t clr_col = 0;
  static uint8_t clr_row = 0;

  osd_transaction_t *txn = osd_txn_init();
  osd_txn_start(OSD_ATTR_TEXT, clr_col, clr_row);
  osd_txn_write_str("               ");
  osd_txn_submit(txn);

  clr_row++;
  if (clr_row > MAXROWS) {
    clr_row = 0;
    clr_col += 15;
    if (clr_col > 15) {
      clr_col = 0;
      return 1;
    }
  }

  return 0;
}

// function to detect and correct ntsc/pal mode or mismatch
// returns the current system
osd_system_t max7456_check_system() {
  if (last_osd_system > OSD_SYS_NONE) {
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
      osd_clear();
    } else if (warning_sent == 1) {
      spi_txn_wait(&bus);

      osd_transaction_t *txn = osd_txn_init();
      osd_txn_start(OSD_ATTR_BLINK, SYSTEMXPOS, SYSTEMYPOS);
      osd_txn_write_str("NO CAMERA SIGNAL");
      osd_txn_submit(txn);

      spi_txn_wait(&bus);
    } else if (warning_sent > 1) {
      // done with this sequence, do not increment further
      break;
    }

    warning_sent++;
    break;
  }
  }

  return current_osd_system;
}

// splash screen
void max7456_intro() {
  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }

    osd_transaction_t *txn = osd_txn_init();
    osd_txn_start(OSD_ATTR_TEXT, 3, row + 5);
    osd_txn_write_data(buffer, 24);
    osd_txn_submit(txn);

    spi_txn_wait(&bus);
  }
}

void max7456_txn_start(osd_transaction_t *txn, uint8_t attr, uint8_t x, uint8_t y) {
  if (dma_offset > 0) {
    // off autoincrement mode
    dma_buffer[dma_offset++] = DMDI;
    dma_buffer[dma_offset++] = 0xFF;
  }

  // NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
  if (last_osd_system != OSD_SYS_PAL && y > 12) {
    y = y - 2;
  }
  if (y > MAXROWS - 1) {
    y = MAXROWS - 1;
  }

  const uint16_t pos = x + y * 30;

  dma_buffer[dma_offset++] = DMM;
  dma_buffer[dma_offset++] = max7456_map_attr(attr);
  dma_buffer[dma_offset++] = DMAH;
  dma_buffer[dma_offset++] = (pos >> 8) & 0xFF;
  dma_buffer[dma_offset++] = DMAL;
  dma_buffer[dma_offset++] = pos & 0xFF;
}

void max7456_txn_write_char(osd_transaction_t *txn, const char val) {
  dma_buffer[dma_offset++] = DMDI;
  dma_buffer[dma_offset++] = val;
}

void max7456_txn_write_data(osd_transaction_t *txn, const uint8_t *buffer, uint8_t size) {
  for (uint8_t i = 0; i < size; i++) {
    max7456_txn_write_char(txn, buffer[i]);
  }
}

void max7456_txn_submit(osd_transaction_t *txn) {
  // off autoincrement mode
  dma_buffer[dma_offset++] = DMDI;
  dma_buffer[dma_offset++] = 0xFF;

  max7456_dma_it_transfer_bytes(dma_buffer, dma_offset);
  dma_offset = 0;
}

bool max7456_is_ready() {
  return spi_txn_ready(&bus);
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
  while (max7456_dma_spi_read(STAT) & 0x20)
    ;

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
  while (max7456_dma_spi_read(STAT) & 0x20)
    ;

  // enable osd
  max7456_init_display();
}

#endif
