#include "drv_spi_max7456.h"

#include <stdio.h>

#include "drv_osd.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"
#include "string.h"
#include "util.h"

#ifdef ENABLE_OSD

#define MAX7456_BAUD_RATE spi_find_divder(MHZ_TO_HZ(10.5))

// osd video system ( PAL /NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
static uint8_t osdsystem = NTSC;

// detected osd video system starts at 99 and gets updated here by osd_checksystem()
uint8_t lastsystem = 99;
static uint8_t lastvm0 = 0x55;

static uint8_t dma_buffer[64];

// TODO ... should we monitor lastvm0 and handle any unexpected changes using check_osd() ... not sure if/when an osd chip becomes unstable due to voltage or some other reason

static volatile uint8_t buffer[128];
static volatile spi_bus_device_t bus = {
    .port = MAX7456_SPI_PORT,
    .nss = MAX7456_NSS,

    .buffer = buffer,
    .buffer_size = 128,

    .auto_continue = true,
};

// blocking dma read of a single register
static uint8_t max7456_dma_spi_read(uint8_t reg) {
  spi_bus_device_reconfigure(&bus, true, MAX7456_BAUD_RATE);

  uint8_t buffer[2] = {reg, 0xFF};

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, buffer, buffer, 2);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);

  return buffer[1];
}

// blocking dma write of a single register
static void max7456_dma_spi_write(uint8_t reg, uint8_t data) {
  spi_bus_device_reconfigure(&bus, true, MAX7456_BAUD_RATE);

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg_const(txn, reg);
  spi_txn_add_seg_const(txn, data);
  spi_txn_submit(txn);

  spi_txn_wait(&bus);
}

// establish initial boot-up state
void max7456_init() {
  spi_bus_device_init(&bus);
  spi_bus_device_reconfigure(&bus, true, MAX7456_BAUD_RATE);

  max7456_dma_spi_write(VM0, 0x02); // soft reset
  time_delay_us(200);

  const uint8_t x = max7456_dma_spi_read(OSDBL_R);
  max7456_dma_spi_write(OSDBL_W, x | 0x10);

  if (osdsystem == PAL) {
    max7456_dma_spi_write(VM0, 0x72); // Set pal mode ( ntsc by default) and enable display
    lastvm0 = 0x72;
  } else {
    max7456_dma_spi_write(VM0, 0x08); // Set ntsc mode and enable display
    lastvm0 = 0x08;
  }

  max7456_dma_spi_write(VM1, 0x0C);  // set background brightness (bits 456), blinking time(bits 23), blinking duty cycle (bits 01)
  max7456_dma_spi_write(OSDM, 0x2D); // osd mux & rise/fall ( lowest sharpness)

  osd_checksystem();
}

// non blocking bulk dma transmit for interrupt callback configuration
static void max7456_dma_it_transfer_bytes(const uint8_t *buffer, const uint8_t size) {
  spi_bus_device_reconfigure(&bus, true, MAX7456_BAUD_RATE);

  spi_txn_t *txn = spi_txn_init(&bus, NULL);
  spi_txn_add_seg(txn, NULL, buffer, size);
  spi_txn_submit(txn);

  spi_txn_continue(&bus);
}

//*******************************************************************************OSD FUNCTIONS********************************************************************************

// stuffs a float into a char array.  parameters are array length and precision.  only pads spaces for 0's up to the thousands place.
uint8_t count_digits(uint32_t value) {
  uint8_t count = 0;
  while (value > 0) {
    value /= 10;
    count++;
  }
  return count;
}

// stuffs a float into a char array.  parameters are array length and precision.  only pads spaces for 0's up to the thousands place.
void fast_fprint(uint8_t *str, uint8_t length, float v, uint8_t precision) {
  const uint8_t is_negative = v < 0 ? 1 : 0;

  uint32_t value = v * (is_negative ? -1.01f : 1.0f) * (ipow(10, precision));
  uint8_t digitsinfrontofdecimal = length - (precision + 1);
  static uint32_t last_cast = 0;
  for (uint8_t i = 0; i < digitsinfrontofdecimal; i++) {
    uint32_t cast_value = value / ipow(10, (digitsinfrontofdecimal + (precision - 1) - i));
    str[i] = ((cast_value) - (10 * last_cast)) + 48;
    last_cast = cast_value;
  }

  for (uint8_t i = digitsinfrontofdecimal; i < length; i++) {
    if (i == digitsinfrontofdecimal) {
      if (precision > 0)
        str[i] = 46;
      else
        str[i] = ' ';
    } else {
      uint32_t cast_value = value / ipow(10, (digitsinfrontofdecimal + precision - i));
      str[i] = ((cast_value) - (10 * last_cast)) + 48;
      last_cast = cast_value;
    }
  }
  last_cast = 0;

  if (digitsinfrontofdecimal > 3) {
    if ((str[0] == 48) && (str[1] == 48) && (str[2] == 48))
      str[2] = is_negative ? '-' : ' ';
    if ((str[0] == 48) && (str[1] == 48))
      str[1] = ' ';
    if (str[0] == 48)
      str[0] = ' ';
  }
  if (digitsinfrontofdecimal > 2) {
    if ((str[0] == 48) && (str[1] == 48))
      str[1] = is_negative ? '-' : ' ';
    if (str[0] == 48)
      str[0] = ' ';
  }
  if (digitsinfrontofdecimal > 1) {
    if (str[0] == 48)
      str[0] = is_negative ? '-' : ' ';
  }
}

// prints array to screen with array length, dmm_attribute TEXT, BLINK, or INVERT, and xy position
void osd_print_data(const uint8_t *buffer, uint8_t length, uint8_t dmm_attribute, uint8_t x, uint8_t y) {
  if (lastsystem != PAL) {
    // NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
    if (y > 12)
      y = y - 2;
  }
  if (y > MAXROWS - 1)
    y = MAXROWS - 1;

  const uint32_t size = (length * 2) + 8;
  if (size > 64) {
    return;
  }

  // 16 bit mode, auto increment mode
  uint16_t pos = x + y * 30;
  dma_buffer[0] = DMM;
  dma_buffer[1] = dmm_attribute;
  dma_buffer[2] = DMAH;
  dma_buffer[3] = (pos >> 8) & 0xFF;
  dma_buffer[4] = DMAL;
  dma_buffer[5] = pos & 0xFF;

  for (int i = 0; i < length; i++) {
    dma_buffer[(i * 2) + 6] = DMDI;
    dma_buffer[(i * 2) + 7] = buffer[i];
  }
  // off autoincrement mode
  dma_buffer[(length * 2) + 6] = DMDI;
  dma_buffer[(length * 2) + 7] = 0xFF;

  // non blocking dma print
  max7456_dma_it_transfer_bytes(dma_buffer, (length * 2) + 8);
}

// prints string to screen with dmm_attribute TEXT, BLINK, or INVERT.  CAUTION:  strlen() is used in this so only use this for compile time strings
void osd_print(const char *buffer, uint8_t dmm_attribute, uint8_t x, uint8_t y) {
  osd_print_data((const uint8_t *)buffer, strlen(buffer), dmm_attribute, x, y);
}

// clears off entire display    This function is a blocking use of non blocking print (not looptime friendly)
void osd_clear() {
  for (uint8_t y = 0; y < MAXROWS; y++) { // CHAR , ATTRIBUTE , COL , ROW
    osd_print("          ", TEXT, 0, y);
    spi_txn_wait(&bus);

    osd_print("          ", TEXT, 10, y);
    spi_txn_wait(&bus);

    osd_print("          ", TEXT, 20, y);
    spi_txn_wait(&bus);
  }
}

uint8_t osd_runtime_screen_clear() {
  static uint8_t clr_col = 0;
  static uint8_t clr_row = 0;
  osd_print("               ", TEXT, clr_col, clr_row);
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

// set the video output system PAL /NTSC
void osd_setsystem(uint8_t sys) {
  uint8_t x = max7456_dma_spi_read(VM0_R);
  if (sys == PAL) {
    lastvm0 = x | 0x40;
    max7456_dma_spi_write(VM0, x | 0x40);
  } else {
    lastvm0 = x & 0xBF;
    max7456_dma_spi_write(VM0, x & 0xBF);
  }
}

// function to autodetect and correct ntsc/pal mode or mismatch
void osd_checksystem() {
  // check detected video system
  uint8_t x = max7456_dma_spi_read(STAT);
  if ((x & 0x01) == 0x01) { // PAL
    if (lastsystem != PAL) {
      lastsystem = PAL;
      if (osdsystem != PAL)
        osd_setsystem(PAL);
      osd_clear(); // initial screen clear off
                   // osd_print( "PAL  DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }
  }

  if ((x & 0x02) == 0x02) { // NTSC
    if (lastsystem != NTSC) {
      lastsystem = NTSC;
      if (osdsystem != NTSC)
        osd_setsystem(NTSC);
      osd_clear(); // initial screen clear off
                   // osd_print( "NTSC DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }
  }

  if ((x & 0x03) == 0x00) { // No signal
    if (lastsystem > 1) {
      if (lastsystem > 2)
        osd_clear(); // initial screen clear off since lastsystem is set to 99 at boot
      static uint8_t warning_sent = 0;
      if (warning_sent < 2) // incriments once at boot, and again the first time through main loop.  Then cleared by a incoming signal
      {
        if (warning_sent == 1)
          osd_print("NO CAMERA SIGNAL", BLINK, SYSTEMXPOS, SYSTEMYPOS);

        spi_txn_wait(&bus);
        warning_sent++;
        lastsystem = 2;
      }
    }
  }
}

// splash screen
void osd_intro() {
  uint8_t buffer[24];
  for (uint8_t row = 0; row < 4; row++) {
    uint8_t start = 160 + row * 24;
    for (uint8_t i = 0; i < 24; i++) {
      buffer[i] = start + i;
    }
    osd_print_data(buffer, 24, TEXT, 3, row + 5);
    spi_txn_wait(&bus);
  }
}

// NOT USING THIS FUNCTION YET OR EVEN SURE IF IT IS NEEDED
//  check for osd "accidental" reset
//  possibly caused by low or unstable voltage
//  MAX resets somewhere between 4.2V and 4.6V
//   Clone chips are unknown to me but obviously below 3.3v
void check_osd() {
  uint8_t x = max7456_dma_spi_read(VM0_R);
  if (x != lastvm0) {                 // the register is not what it's supposed to be
    max7456_dma_spi_write(VM0, 0x02); // soft reset
    time_delay_us(200);
    // only set minimum number of registers for functionality
    if (osdsystem == PAL) {
      max7456_dma_spi_write(VM0, 0x72); // Set pal mode ( ntsc by default) and enable display
      lastvm0 = 0x72;
    } else {
      max7456_dma_spi_write(VM0, 0x08); // Set ntsc mode and enable display
      lastvm0 = 0x08;
    }
  }
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
  max7456_dma_spi_write(VM0, 0x1);
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
  max7456_dma_spi_write(VM0, 0x1);
}

void osd_txn_submit(osd_transaction_t *txn) {
  uint16_t offset = 0;

  for (uint16_t i = 0; i < txn->segment_count; i++) {
    osd_segment_t *seg = &txn->segments[i];

    // NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
    if (lastsystem != PAL && seg->y > 12) {
      seg->y = seg->y - 2;
    }
    if (seg->y > MAXROWS - 1) {
      seg->y = MAXROWS - 1;
    }

    const uint16_t pos = seg->x + seg->y * 30;

    dma_buffer[offset++] = DMM;
    dma_buffer[offset++] = seg->attr;
    dma_buffer[offset++] = DMAH;
    dma_buffer[offset++] = (pos >> 8) & 0xFF;
    dma_buffer[offset++] = DMAL;
    dma_buffer[offset++] = pos & 0xFF;

    for (uint16_t j = 0; j < seg->size; j++) {
      dma_buffer[offset++] = DMDI;
      dma_buffer[offset++] = txn->buffer[seg->offset + j];
    }

    // off autoincrement mode
    dma_buffer[offset++] = DMDI;
    dma_buffer[offset++] = 0xFF;
  }

  max7456_dma_it_transfer_bytes(dma_buffer, offset);
}

bool osd_is_ready() {
  return spi_txn_ready(&bus);
}

#endif
