#include "drv_spi_max7456.h"

#include <stdio.h>

#include "defines.h"
#include "drv_spi.h"
#include "drv_time.h"
#include "project.h"
#include "string.h"
#include "util.h"

#ifdef ENABLE_OSD

//SPI PINS
#define PORT spi_port_defs[MAX7456_SPI_PORT]

#define DMA_RX_STREAM PORT.dma.rx_stream
#define DMA_TX_STREAM PORT.dma.tx_stream
#define DMA_RX_CHANNEL PORT.dma.channel
#define DMA_TX_CHANNEL PORT.dma.channel
#define DMA_RX_TCI_FLAG PORT.dma.rx_tci_flag
#define DMA_TX_TCI_FLAG PORT.dma.tx_tci_flag
#define DMA_RX_STREAM_IRQ PORT.dma.rx_it
#define DMA_RX_IT_FLAG PORT.dma.rx_it_flag

//  Initialize SPI Connection to max7456
void spi_max7456_init(void) {

  //*********************GPIO**************************************
  spi_init_pins(MAX7456_SPI_PORT, MAX7456_NSS);

  //*********************SPI/DMA**********************************
  spi_enable_rcc(MAX7456_SPI_PORT);

  // SPI Config
  SPI_I2S_DeInit(PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(PORT.channel, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(PORT.channel);

  spi_dma_init(MAX7456_SPI_PORT);
}

//deinit/reinit spi for unique slave configuration
void spi_max7556_reinit(void) {
  spi_dma_wait_for_ready(MAX7456_SPI_PORT);

  SPI_Cmd(PORT.channel, DISABLE);

  // SPI Config
  SPI_I2S_DeInit(PORT.channel);
  SPI_InitTypeDef SPI_InitStructure;
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(PORT.channel, &SPI_InitStructure);
  SPI_Cmd(PORT.channel, ENABLE);
}

//*******************************************************************************SPI / DMA FUNCTIONS********************************************************************************

#define BUSY 1
#define READY 0
volatile uint8_t osd_dma_status = READY; //for tracking the non blocking dma transactions - can be used to make non blocking into blocking

// blocking dma read of a single register
uint8_t max7456_dma_spi_read(uint8_t reg) {
  spi_max7556_reinit();

  uint8_t buffer[2] = {reg, 0xFF};

  spi_csn_enable(MAX7456_NSS);
  spi_dma_transfer_bytes(MAX7456_SPI_PORT, buffer, 2);
  spi_csn_disable(MAX7456_NSS);

  return buffer[1];
}

// blocking dma write of a single register
void max7456_dma_spi_write(uint8_t reg, uint8_t data) {
  spi_max7556_reinit();

  uint8_t buffer[2] = {reg, data};
  spi_csn_enable(MAX7456_NSS);
  spi_dma_transfer_bytes(MAX7456_SPI_PORT, buffer, 2);
  spi_csn_disable(MAX7456_NSS);
}

// non blocking bulk dma transmit for interrupt callback configuration
void max7456_dma_it_transfer_bytes(uint8_t *buffer_address, uint8_t buffer_length) {
  osd_dma_status = BUSY;
  spi_max7556_reinit();

  spi_csn_enable(MAX7456_NSS);
  spi_dma_transfer_begin(MAX7456_SPI_PORT, buffer_address, buffer_length);
}

// callback function to disable csn
void max7456_dma_rx_isr() {
  spi_csn_disable(MAX7456_NSS);
  osd_dma_status = READY;
}

//*******************************************************************************OSD FUNCTIONS********************************************************************************

// osd video system ( PAL /NTSC) at startup if no video input is present
// after input is present the last detected system will be used.
uint8_t osdsystem = NTSC;
// detected osd video system starts at 99 and gets updated here by osd_checksystem()
uint8_t lastsystem = 99;
uint8_t lastvm0 = 0x55;

//TODO ... should we monitor lastvm0 and handle any unexpected changes using check_osd() ... not sure if/when an osd chip becomes unstable due to voltage or some other reason

//stuffs a float into a char array.  parameters are array length and precision.  only pads spaces for 0's up to the thousands place.

uint8_t count_digits(uint32_t value) {
  uint8_t count = 0;
  while (value > 0) {
    value /= 10;
    count++;
  }
  return count;
}

//stuffs a float into a char array.  parameters are array length and precision.  only pads spaces for 0's up to the thousands place.
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
void osd_print_data(uint8_t *buffer, uint8_t length, uint8_t dmm_attribute, uint8_t x, uint8_t y) {
  if (lastsystem != PAL) {
    //NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
    if (y > 12)
      y = y - 2;
  }
  if (y > MAXROWS - 1)
    y = MAXROWS - 1;

  const uint32_t size = (length * 2) + 8;
  static uint8_t osd_buffer[18]; //allows for a maximum of 5 characters to be written to screen per loop
  if (size > 18) {
    return;
  }

  // 16 bit mode, auto increment mode
  uint16_t pos = x + y * 30;
  osd_buffer[0] = DMM;
  osd_buffer[1] = dmm_attribute;
  osd_buffer[2] = DMAH;
  osd_buffer[3] = 0x01 & (pos >> 8);
  osd_buffer[4] = DMAL;
  osd_buffer[5] = (uint8_t)pos;

  for (int i = 0; i < length; i++) {
    osd_buffer[(i * 2) + 6] = DMDI;
    osd_buffer[(i * 2) + 7] = buffer[i];
  }
  // off autoincrement mode
  osd_buffer[(length * 2) + 6] = DMDI;
  osd_buffer[(length * 2) + 7] = 0xFF;
  //non blocking dma print
  max7456_dma_it_transfer_bytes(osd_buffer, (length * 2) + 8);
}

// prints string to screen with dmm_attribute TEXT, BLINK, or INVERT.  CAUTION:  strlen() is used in this so only use this for compile time strings
void osd_print(const char *buffer, uint8_t dmm_attribute, uint8_t x, uint8_t y) {
  if (lastsystem != PAL) {
    //NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
    if (y > 12)
      y = y - 2;
  }
  if (y > MAXROWS - 1)
    y = MAXROWS - 1;

  const uint32_t size = (strlen(buffer) * 2) + 8;
  static uint8_t osd_string_buffer[48]; //allows for a maximum of 20 characters to be written to screen per loop
  if (size > 48) {
    return;
  }

  // 16 bit mode, auto increment mode
  uint16_t pos = x + y * 30;
  osd_string_buffer[0] = DMM;
  osd_string_buffer[1] = dmm_attribute;
  osd_string_buffer[2] = DMAH;
  osd_string_buffer[3] = 0x01 & (pos >> 8);
  osd_string_buffer[4] = DMAL;
  osd_string_buffer[5] = (uint8_t)pos;

  for (unsigned int i = 0; i < strlen(buffer); i++) {
    osd_string_buffer[(i * 2) + 6] = DMDI;
    osd_string_buffer[(i * 2) + 7] = buffer[i];
  }
  // off autoincrement mode
  osd_string_buffer[(strlen(buffer) * 2) + 6] = DMDI;
  osd_string_buffer[(strlen(buffer) * 2) + 7] = 0xFF;

  //non blocking dma print
  max7456_dma_it_transfer_bytes(osd_string_buffer, size);
}

//clears off entire display    This function is a blocking use of non blocking print (not looptime friendly)
void osd_clear(void) {
  for (uint8_t y = 0; y < MAXROWS; y++) { // CHAR , ATTRIBUTE , COL , ROW
    osd_print("          ", TEXT, 0, y);
    while (osd_dma_status == BUSY) {
    };
    osd_print("          ", TEXT, 10, y);
    while (osd_dma_status == BUSY) {
    };
    osd_print("          ", TEXT, 20, y);
    while (osd_dma_status == BUSY) {
    };
  }
}

uint8_t osd_runtime_screen_clear(void) {
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

//function to autodetect and correct ntsc/pal mode or mismatch
void osd_checksystem(void) {
  //check detected video system
  uint8_t x = max7456_dma_spi_read(STAT);
  if ((x & 0x01) == 0x01) { //PAL
    if (lastsystem != PAL) {
      lastsystem = PAL;
      if (osdsystem != PAL)
        osd_setsystem(PAL);
      osd_clear(); // initial screen clear off
                   //osd_print( "PAL  DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }
  }

  if ((x & 0x02) == 0x02) { //NTSC
    if (lastsystem != NTSC) {
      lastsystem = NTSC;
      if (osdsystem != NTSC)
        osd_setsystem(NTSC);
      osd_clear(); // initial screen clear off
                   //osd_print( "NTSC DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }
  }

  if ((x & 0x03) == 0x00) { //No signal
    if (lastsystem > 1) {
      if (lastsystem > 2)
        osd_clear(); // initial screen clear off since lastsystem is set to 99 at boot
      static uint8_t warning_sent = 0;
      if (warning_sent < 2) //incriments once at boot, and again the first time through main loop.  Then cleared by a incoming signal
      {
        if (warning_sent == 1)
          osd_print("NO CAMERA SIGNAL", BLINK, SYSTEMXPOS, SYSTEMYPOS);
        while (osd_dma_status == BUSY) {
        };
        warning_sent++;
        lastsystem = 2;
      }
    }
  }
}

//establish initial boot-up state
void max7456_init(void) {
  uint8_t x;
  max7456_dma_spi_write(VM0, 0x02); //soft reset
  timer_delay_us(200);
  x = max7456_dma_spi_read(OSDBL_R);
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

//splash screen
void osd_intro(void) {
  osd_print("QUICKSILVER", INVERT, 9, 5); //char, col, row
  while (osd_dma_status == BUSY) {
  };
  osd_print("LIKE A BOSS", TEXT, 18, 14); //char, col, row
  while (osd_dma_status == BUSY) {
  }; //this wait may not be necessary
}

//NOT USING THIS FUNCTION YET OR EVEN SURE IF IT IS NEEDED
// check for osd "accidental" reset
// possibly caused by low or unstable voltage
// MAX resets somewhere between 4.2V and 4.6V
//  Clone chips are unknown to me but obviously below 3.3v
void check_osd(void) {
  uint8_t x = max7456_dma_spi_read(VM0_R);
  if (x != lastvm0) {                 // the register is not what it's supposed to be
    max7456_dma_spi_write(VM0, 0x02); // soft reset
    timer_delay_us(200);
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
  while (osd_dma_status == BUSY)
    ;

  // disable osd
  max7456_dma_spi_write(VM0, 0x0);
  timer_delay_us(10);

  max7456_dma_spi_write(CMAH, addr);
  max7456_dma_spi_write(CMM, 0x50);

  // wait for NVM to be ready
  while (max7456_dma_spi_read(STAT) & 0x20)
    ;

  for (uint8_t i = 0; i < size; i++) {
    max7456_dma_spi_write(CMAL, i);
    out[i] = max7456_dma_spi_read(CMDO);
    timer_delay_us(1);
  }

  // enable osd
  max7456_dma_spi_write(VM0, 0x1);
}

void osd_write_character(uint8_t addr, const uint8_t *in, const uint8_t size) {
  // make sure we do not collide with a dma write
  while (osd_dma_status == BUSY)
    ;

  // disable osd
  max7456_dma_spi_write(VM0, 0x0);
  timer_delay_us(10);

  max7456_dma_spi_write(CMAH, addr);
  for (uint8_t i = 0; i < size; i++) {
    max7456_dma_spi_write(CMAL, i);
    max7456_dma_spi_write(CMDI, in[i]);
    timer_delay_us(1);
  }

  max7456_dma_spi_write(CMM, 0xA0);

  // wait for NVM to be ready
  while (max7456_dma_spi_read(STAT) & 0x20)
    ;

  // enable osd
  max7456_dma_spi_write(VM0, 0x1);
}

#endif
