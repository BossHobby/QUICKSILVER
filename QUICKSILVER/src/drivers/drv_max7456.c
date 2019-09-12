#include "drv_max7456.h"

#include <stdio.h>

#include "defines.h"
#include "drv_time.h"
#include "project.h"
#include "string.h"
#include "util.h"

#ifdef ENABLE_OSD

//SPI PINS
#ifdef MAX7456_SPI2
#define RCC_APBPeriph_SPI RCC_APB1Periph_SPI2
#define MAX7456_SPI_INSTANCE SPI2
#define MAX7456_SPI_PORT GPIOB
#define MAX7456_SCLK_PINSOURCE GPIO_PinSource13
#define MAX7456_SCLK_PIN GPIO_Pin_13
#define MAX7456_MISO_PINSOURCE GPIO_PinSource14
#define MAX7456_MISO_PIN GPIO_Pin_14
#define MAX7456_MOSI_PINSOURCE GPIO_PinSource15
#define MAX7456_MOSI_PIN GPIO_Pin_15
#define MAX7456_SPI_AF GPIO_AF_SPI2
#define DMA1_RX_STREAM DMA1_Stream3
#define DMA1_TX_STREAM DMA1_Stream4
#define DMA_RX_CHANNEL DMA_Channel_0
#define DMA_TX_CHANNEL DMA_Channel_0
#define DMA_RX_TCI_FLAG DMA_FLAG_TCIF3
#define DMA_TX_TCI_FLAG DMA_FLAG_TCIF4
#define DMA_STREAM_IRQ DMA1_Stream3_IRQn
#define DMA_RX_IT_FLAG DMA_IT_TCIF3
#endif

#ifdef MAX7456_SPI3
#define RCC_APBPeriph_SPI RCC_APB1Periph_SPI3
#define MAX7456_SPI_INSTANCE SPI3
#define MAX7456_SPI_PORT GPIOC
#define MAX7456_SCLK_PINSOURCE GPIO_PinSource10
#define MAX7456_SCLK_PIN GPIO_Pin_10
#define MAX7456_MISO_PINSOURCE GPIO_PinSource11
#define MAX7456_MISO_PIN GPIO_Pin_11
#define MAX7456_MOSI_PINSOURCE GPIO_PinSource12
#define MAX7456_MOSI_PIN GPIO_Pin_12
#define MAX7456_SPI_AF GPIO_AF_SPI3
#define DMA1_RX_STREAM DMA1_Stream0
#define DMA1_TX_STREAM DMA1_Stream7
#define DMA_RX_CHANNEL DMA_Channel_0
#define DMA_TX_CHANNEL DMA_Channel_0
#define DMA_RX_TCI_FLAG DMA_FLAG_TCIF0
#define DMA_TX_TCI_FLAG DMA_FLAG_TCIF7
#define DMA_STREAM_IRQ DMA1_Stream0_IRQn
#define DMA_RX_IT_FLAG DMA_IT_TCIF0
#endif

//NSS PINS
#ifdef MAX7456_NSS_PD2
#define MAX7456_NSS_PIN GPIO_Pin_2
#define MAX7456_NSS_PORT GPIOD
#endif

#ifdef MAX7456_NSS_PB12
#define MAX7456_NSS_PIN GPIO_Pin_12
#define MAX7456_NSS_PORT GPIOB
#endif

#ifdef MAX7456_NSS_PB3
#define MAX7456_NSS_PIN GPIO_Pin_3
#define MAX7456_NSS_PORT GPIOB
#endif

#ifdef MAX7456_NSS_PA15
#define MAX7456_NSS_PIN GPIO_Pin_15
#define MAX7456_NSS_PORT GPIOA
#endif

//  Initialize SPI Connection to max7456
void spi_max7456_init(void) {

  //*********************GPIO**************************************

  // GPIO & Alternate Function Setting
  GPIO_InitTypeDef GPIO_InitStructure;
  // Clock, Miso, Mosi GPIO
  GPIO_InitStructure.GPIO_Pin = MAX7456_SCLK_PIN | MAX7456_MISO_PIN | MAX7456_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(MAX7456_SPI_PORT, &GPIO_InitStructure);

  // Chip Select GPIO
  GPIO_InitStructure.GPIO_Pin = MAX7456_NSS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(MAX7456_NSS_PORT, &GPIO_InitStructure);

  // Chip Select Set High
  GPIO_SetBits(MAX7456_NSS_PORT, MAX7456_NSS_PIN);

  // Connect SPI pins to AF_SPI1
  GPIO_PinAFConfig(MAX7456_SPI_PORT, MAX7456_SCLK_PINSOURCE, MAX7456_SPI_AF); //SCLK
  GPIO_PinAFConfig(MAX7456_SPI_PORT, MAX7456_MISO_PINSOURCE, MAX7456_SPI_AF); //MISO
  GPIO_PinAFConfig(MAX7456_SPI_PORT, MAX7456_MOSI_PINSOURCE, MAX7456_SPI_AF); //MOSI

  //*********************SPI / DMA***************************************

  //SPI1 to APB2 bus clock
  RCC_APB1PeriphClockCmd(RCC_APBPeriph_SPI, ENABLE);
  // SPI Config
  SPI_I2S_DeInit(MAX7456_SPI_INSTANCE);
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
  SPI_Init(MAX7456_SPI_INSTANCE, &SPI_InitStructure);
  SPI_Cmd(MAX7456_SPI_INSTANCE, ENABLE);

  // Dummy read to clear receive buffer
  while (SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
    ;
  SPI_I2S_ReceiveData(MAX7456_SPI_INSTANCE);

  // Enable DMA clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  // Enable DMA Interrupt on receive line
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = DMA_STREAM_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_Init(&NVIC_InitStruct);
}

// dma stream inits
void dma_receive_max7456_init(uint8_t *base_address_in, uint8_t buffer_size) {
  DMA_InitTypeDef DMA_InitStructure;

  //RX Stream
  DMA_DeInit(DMA1_RX_STREAM);
  DMA_InitStructure.DMA_Channel = DMA_RX_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(MAX7456_SPI_INSTANCE->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_in;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  ;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_RX_STREAM, &DMA_InitStructure);
}

void dma_transmit_max7456_init(uint8_t *base_address_out, uint8_t buffer_size) {
  DMA_InitTypeDef DMA_InitStructure;

  //TX Stream
  DMA_DeInit(DMA1_TX_STREAM);
  DMA_InitStructure.DMA_Channel = DMA_TX_CHANNEL;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(MAX7456_SPI_INSTANCE->DR));
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)base_address_out;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_BufferSize = (uint16_t)buffer_size;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_TX_STREAM, &DMA_InitStructure);
}

//*******************************************************************************SPI / DMA FUNCTIONS********************************************************************************

int osd_liberror; //tracks any failed spi reads or writes to trigger check_osd() on next disarm or maybe print an osd fail warning?  Not used yet
#define BUSY 1
#define READY 0
volatile uint8_t osd_dma_status = READY; //for tracking the non blocking dma transactions - can be used to make non blocking into blocking

// Chip Select functions
void max7456_enable() {
  GPIO_ResetBits(MAX7456_NSS_PORT, MAX7456_NSS_PIN);
}

void max7456_disable() {
  GPIO_SetBits(MAX7456_NSS_PORT, MAX7456_NSS_PIN);
}

//blocking dma transmit bytes
void max7456_dma_transfer_bytes(uint8_t *buffer, uint8_t length) {
  dma_receive_max7456_init(buffer, length);
  dma_transmit_max7456_init(buffer, length);
  max7456_enable();
  DMA_Cmd(DMA1_RX_STREAM, ENABLE); // Enable the DMA SPI RX Stream
  DMA_Cmd(DMA1_TX_STREAM, ENABLE); // Enable the DMA SPI TX Stream
  // Enable the SPI Rx/Tx DMA request
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_Cmd(MAX7456_SPI_INSTANCE, ENABLE);
  /* Waiting the end of Data transfer */
  while (DMA_GetFlagStatus(DMA1_RX_STREAM, DMA_RX_TCI_FLAG) == RESET) {
  };
  while (DMA_GetFlagStatus(DMA1_TX_STREAM, DMA_TX_TCI_FLAG) == RESET) {
  };
  DMA_ClearFlag(DMA1_RX_STREAM, DMA_RX_TCI_FLAG);
  DMA_ClearFlag(DMA1_TX_STREAM, DMA_TX_TCI_FLAG);
  DMA_Cmd(DMA1_TX_STREAM, DISABLE);
  DMA_Cmd(DMA1_RX_STREAM, DISABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_Cmd(MAX7456_SPI_INSTANCE, DISABLE);
  max7456_disable();
}

// blocking dma read of a single register
uint8_t max7456_dma_spi_read(uint8_t reg) {
  uint8_t buffer[2] = {reg, 0xFF};
  max7456_dma_transfer_bytes(buffer, 2);
  return buffer[1];
}

// blocking dma write of a single register
void max7456_dma_spi_write(uint8_t reg, uint8_t data) {
  uint8_t buffer[2] = {reg, data};
  max7456_dma_transfer_bytes(buffer, 2);
}

// non blocking bulk dma transmit for interrupt callback configuration
void max7456_dma_it_transfer_bytes(uint8_t *buffer_address, uint8_t buffer_length) {
  osd_dma_status = BUSY;
  dma_transmit_max7456_init(buffer_address, buffer_length);
  dma_receive_max7456_init(buffer_address, buffer_length);
  DMA_ITConfig(DMA1_RX_STREAM, DMA_IT_TC, ENABLE); // Configure the Interrupt on transfer complete for either SPI tx or rx
  max7456_enable();
  DMA_Cmd(DMA1_RX_STREAM, ENABLE); // Enable the DMA SPI RX Stream
  DMA_Cmd(DMA1_TX_STREAM, ENABLE); // Enable the DMA SPI TX Stream
                                   // Enable the SPI Rx/Tx DMA request
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, ENABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, ENABLE);
  SPI_Cmd(MAX7456_SPI_INSTANCE, ENABLE);
}

//callback function to shut down dma streams
void transfer_complete_cb(void) {
  DMA_ClearFlag(DMA1_TX_STREAM, DMA_TX_TCI_FLAG);
  DMA_ClearFlag(DMA1_RX_STREAM, DMA_RX_TCI_FLAG);
  DMA_Cmd(DMA1_TX_STREAM, DISABLE);
  DMA_Cmd(DMA1_RX_STREAM, DISABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Tx, DISABLE);
  SPI_I2S_DMACmd(MAX7456_SPI_INSTANCE, SPI_I2S_DMAReq_Rx, DISABLE);
  SPI_Cmd(MAX7456_SPI_INSTANCE, DISABLE);
  max7456_disable();
}

//irq handler for receive interrupt
#ifdef MAX7456_SPI3
void DMA1_Stream0_IRQHandler(void)
#endif
#ifdef MAX7456_SPI2
    void DMA1_Stream3_IRQHandler(void)
#endif
{
  if (DMA_GetITStatus(DMA1_RX_STREAM, DMA_RX_IT_FLAG)) {
    DMA_ClearITPendingBit(DMA1_RX_STREAM, DMA_RX_IT_FLAG);
    DMA_ITConfig(DMA1_RX_STREAM, DMA_IT_TC, DISABLE);
    transfer_complete_cb();
    osd_dma_status = READY;
  }
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

  // make sure our string is empty
  memset(str, 0, length);

  // calculate what we want to multiply by
  const uint32_t multiplier = ipow(10, precision);
  const uint8_t digits = count_digits(v);
  const uint32_t padding = length - (digits + precision + 1);

  // move our decimal point
  uint32_t value = v * multiplier;
  uint32_t divider = ipow(10, digits + precision - 1);

  for (uint32_t i = 0; i < length; ++i) {
    if (i < padding) {
      str[i] = ' ';
      continue;
    }

    if (value <= 0) {
      if ((i - digits) <= precision) {
        str[i] = '0';
        continue;
      }
      break;
    }

    if (i == padding + digits) {
      str[i] = '.';
      continue;
    }

    uint8_t digit = value / divider;
    str[i] = '0' + digit;
    value = value % divider;
    divider /= 10;
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
void osd_print(char *buffer, uint8_t dmm_attribute, uint8_t x, uint8_t y) {
  if (lastsystem != PAL) {
    //NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
    if (y > 12)
      y = y - 2;
  }
  if (y > MAXROWS - 1)
    y = MAXROWS - 1;

  const uint32_t size = (strlen(buffer) * 2) + 8;
  static uint8_t osd_string_buffer[40]; //allows for a maximum of 16 characters to be written to screen per loop
  if (size > 40) {
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
  delay(200);
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
  osd_print("BY ALIENWHOOP", TEXT, 16, 14); //char, col, row
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
    delay(200);
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

#endif
