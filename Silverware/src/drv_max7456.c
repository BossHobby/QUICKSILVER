

#include "project.h"
#include <stdio.h>
#include "defines.h"
#include "drv_time.h"
#include "drv_max7456.h"
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


//  Initialize SPI Connection to Gyro
void spi_max7456_init(void)
{
	
//*********************GPIO**************************************	
	
// GPIO & Alternate Function Setting
GPIO_InitTypeDef    GPIO_InitStructure;
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

	
//*********************SPI***************************************	


//SPI1 to APB2 bus clock
RCC_APB1PeriphClockCmd( RCC_APBPeriph_SPI, ENABLE);
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
while(SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET) ;
SPI_I2S_ReceiveData(MAX7456_SPI_INSTANCE);
}


//*******************************************************************************SPI FUNCTIONS********************************************************************************
extern int liberror;   //tracks any failed spi reads or writes to trigger failloop



// Chip Select functions
void max7456_enable()
{
	GPIO_ResetBits(MAX7456_NSS_PORT, MAX7456_NSS_PIN);
}

void max7456_disable()
{
	GPIO_SetBits(MAX7456_NSS_PORT, MAX7456_NSS_PIN); 
}



// Blocking Transmit/Read function
uint8_t max7456_transfer_byte(uint8_t data)
{
  uint16_t spiTimeout;
	
	//check if transmit buffer empty flag is set
  spiTimeout = 0x1000;
  while (SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_TXE) == RESET)
  {
    if ((spiTimeout--) == 0){
			liberror++;										//liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
			break;}
  }
	
	// Send out data
  SPI_I2S_SendData(MAX7456_SPI_INSTANCE, data);

	//wait to receive something ... timeout if nothing comes in 
  spiTimeout = 0x1000;																
  while (SPI_I2S_GetFlagStatus(MAX7456_SPI_INSTANCE, SPI_I2S_FLAG_RXNE) == RESET)
  {
    if ((spiTimeout--) == 0){
			liberror++;										//liberror will trigger failloop 7 during boot, or 20 liberrors will trigger failloop 8 in flight
			break;}
  }

  // Return back received data in SPIx->DR
  return SPI_I2S_ReceiveData(MAX7456_SPI_INSTANCE);
}



// Function to write osd registers
uint8_t MAX7456_write(uint8_t reg, uint8_t data)
{
	uint8_t stuff;
  max7456_enable();
  stuff = max7456_transfer_byte(reg);
  stuff = max7456_transfer_byte(data);
  max7456_disable();
	return stuff;
}


// Function to read osd registers
uint8_t MAX7456_read(uint8_t reg)
{
	uint8_t stuff;
  max7456_enable();
  max7456_transfer_byte(reg);
  stuff = max7456_transfer_byte(0xFF);
  max7456_disable();
	return stuff;
}

//function to auto incriment write - requires handling NSS seperately
void max7456_transfer( uint8_t reg, uint8_t val )
{
  max7456_transfer_byte(reg);
  max7456_transfer_byte(val);
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
void fast_fprint(char* str, uint8_t length, float v, uint8_t precision)
{
	uint32_t value = v * (ipow(10, precision));
	uint8_t digitsinfrontofdecimal = length - (precision + 1);
	static uint32_t last_cast = 0;
	for (uint8_t i = 0; i < digitsinfrontofdecimal; i++) 
	{
	uint32_t cast_value = value / ipow(10, (digitsinfrontofdecimal+(precision-1)-i));
	str[i] = ((cast_value)- (10 * last_cast))+48;
	last_cast = cast_value;
	}

	for (uint8_t i = digitsinfrontofdecimal; i < length; i++)
	{
		if (i == digitsinfrontofdecimal){
		if (precision > 0) str[i] = 46;
		else str[i] = ' ';
		}else{
		uint32_t cast_value = value / ipow(10, (digitsinfrontofdecimal+precision-i));
		str[i] = ((cast_value)- (10 * last_cast))+48;
		last_cast = cast_value;
		}
	}
	last_cast = 0;
	
	if (digitsinfrontofdecimal > 3)
	{
	if ((str[0] == 48) && (str[1] == 48) && (str[2] == 48)) str[2] = ' ';
	if ((str[0] == 48) && (str[1] == 48)) str[1] = ' ';
	if (str[0] == 48) str[0] = ' ';
	}
	if (digitsinfrontofdecimal > 2)
	{
	if ((str[0] == 48) && (str[1] == 48)) str[1] = ' ';
	if (str[0] == 48) str[0] = ' ';
	}
	if (digitsinfrontofdecimal > 1)
	{
	if (str[0] == 48) str[0] = ' ';
	}	
}

// prints char array to screen with array length, dmm_attribute TEXT, BLINK, or INVERT, and xy position
void osd_print_char( char *buffer , uint8_t length,  uint8_t dmm_attribute,  uint8_t x , uint8_t y)
{
  if( lastsystem != PAL)
  {
      //NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
      if ( y > 12 ) y = y - 2;      
  }
  if ( y > MAXROWS-1 ) y = MAXROWS-1;
  
  uint16_t pos = x + y*30; 
				
  max7456_enable();
   // 16 bit mode, auto increment mode 
  max7456_transfer(DMM, dmm_attribute );
  // memory address
  max7456_transfer(DMAH, 0x01 & (pos >> 8) );
  max7456_transfer(DMAL, (uint8_t) pos  );
  
  for ( int i = 0; i < length; i++ ) {
  max7456_transfer( DMDI, buffer[i] );
  }
  // off autoincrement mode
  max7456_transfer( DMDI, 0xFF ); 
  max7456_disable(); 
}

// prints string to screen with dmm_attribute TEXT, BLINK, or INVERT.  CAUTION:  strlen() is used in this so only use this for compile time strings
void osd_print( char *buffer ,  uint8_t dmm_attribute,  uint8_t x , uint8_t y)
{
  if( lastsystem != PAL)
  {
      //NTSC adjustment 3 lines up if after line 12 or maybe this should be 8
      if ( y > 12 ) y = y - 2;      
  }
  if ( y > MAXROWS-1 ) y = MAXROWS-1;
  
  uint16_t pos = x + y*30; 
				
  max7456_enable();
   // 16 bit mode, auto increment mode 
  max7456_transfer(DMM, dmm_attribute );
  // memory address
  max7456_transfer(DMAH, 0x01 & (pos >> 8) );
  max7456_transfer(DMAL, (uint8_t) pos  );
  
  for ( int i = 0; i < strlen(buffer); i++ ) {
  max7456_transfer( DMDI, buffer[i] );
  }
  // off autoincrement mode
  max7456_transfer( DMDI, 0xFF ); 
  max7456_disable(); 
}



//clears off entire display
void osd_clear(void)
{
  for ( uint8_t y = 0 ; y < MAXROWS ; y++)
  {						// CHAR , ATTRIBUTE , COL , ROW
   osd_print( "          " , TEXT,  0 , y ); 
   osd_print( "          " , TEXT,  10 , y ); 
   osd_print( "          " , TEXT,  20 , y ); 
   }
}



// set the video output system PAL /NTSC
void osd_setsystem( uint8_t sys)
{
uint8_t x = MAX7456_read(VM0_R); 
if ( sys == PAL )
    {
     lastvm0 = x | 0x40; 
     MAX7456_write( VM0, x | 0x40 );
    }
else
    {
     lastvm0 =  x & 0xBF;
     MAX7456_write( VM0, x & 0xBF );
    }
}



//function to autodetect and correct ntsc/pal mode or mismatch
void osd_checksystem(void)
{  
  // check detected video system
  uint8_t x = MAX7456_read(STAT);
  if ((x & 0x01) == 0x01)
  { //PAL
   if ( lastsystem != PAL )
    {
    lastsystem = PAL;
    if ( osdsystem != PAL ) 
      {
				osd_setsystem(PAL);
      }
		osd_clear();									 // initial screen clear off
//		osd_print( "PAL  DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }
  } 
  if ((x & 0x02) == 0x02)
  { //NTSC
   if ( lastsystem != NTSC )
    {
    lastsystem = NTSC;
    if ( osdsystem != NTSC ) 
      {
				osd_setsystem(NTSC);				
      }
		osd_clear();									 // initial screen clear off
//		osd_print( "NTSC DETECTED" , BLINK , SYSTEMXPOS+1 , SYSTEMYPOS );  //for debugging - remove later
    }	
  }  
  
   if ( ( x & 0x03 ) == 0x00 )
  {//No signal
   if ( lastsystem > 1 )
    {
		if (lastsystem > 2) osd_clear();									 // initial screen clear off since lastsystem is set to 99 at boot		
		static uint8_t warning_sent = 0;
		if (warning_sent < 2)
			{															//incriments once at boot, and again the first time through main loop.  Then cleared by a incoming signal
			osd_print( "NO CAMERA SIGNAL" , BLINK , SYSTEMXPOS , SYSTEMYPOS );
			warning_sent++;
			lastsystem = 2;
			}
    }
    
  }  
}



//establish initial boot-up state
void max7456_init(void)
{
uint8_t x;
MAX7456_write(VM0 , 0x02);   	 //soft reset
delay(200);
x = MAX7456_read(OSDBL_R); 		 // mine set to 13 (factory) **********check this out
MAX7456_write(OSDBL_W , x|0x10);
if ( osdsystem == PAL )	
  { 
		MAX7456_write(VM0, 0x72);	 // Set pal mode ( ntsc by default) and enable display
		lastvm0 = 0x72;
	}else{
		MAX7456_write(VM0, 0x08);	 // Set ntsc mode and enable display
		lastvm0 = 0x08;
	}	
MAX7456_write(VM1, 0x0C);			 // set background brightness (bits 456), blinking time(bits 23), blinking duty cycle (bits 01) 
MAX7456_write(OSDM, 0x2D);		 // osd mux & rise/fall ( lowest sharpness)

osd_checksystem();

}															 //still need to detect NTSC/PAL here and set placeholder ... maybe more from Silver's functions below too



//splash screen
void osd_intro(void)
{
 osd_print( "QUICKSILVER" , INVERT ,  9  , 5	);  //char, col, row
 osd_print( "BY ALIENWHOOP" , TEXT ,  16 , 14	);  //char, col, row
}



//NOT USING THIS FUNCTION YET OR EVEN SURE IF IT IS NEEDED
// check for osd "accidental" reset
// the MAX7456 and atmega328 have very different voltage ranges
// MAX resets somewhere between 4.2V and 4.6V
// atmega328 works to below 3.3V (depending on brownout fuses)
void check_osd(void)
{
  uint8_t x = MAX7456_read(VM0_R);   
  if ( x != lastvm0)
  {// the register is not what it's supposed to be
   MAX7456_write( VM0, 0x02 ); // soft reset
   delay(200);
   // only set minimum number of registers for functionality   
   if ( osdsystem == PAL )
  { 
		MAX7456_write(VM0, 0x72);	 // Set pal mode ( ntsc by default) and enable display
		lastvm0 = 0x72;
	}else{
		MAX7456_write(VM0, 0x08);	 // Set ntsc mode and enable display
		lastvm0 = 0x08;
	}	
  } 
}


#endif



