//#define TESTING_OSD
#ifdef TESTING_OSD
#include "project.h"
#include <stdio.h>
#include "defines.h"
#include "drv_time.h"
#include "drv_max7456.h"


#define MAX7456_SPI2
#define MAX7456_NSS_PD2

//SPI PINS
#ifdef MAX7456_SPI2
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

#ifdef MAX7456_NSS_PD2
#define MAX7456_NSS_PIN GPIO_Pin_2
#define MAX7456_NSS_PORT GPIOD
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


//SPI1 to APB2 bus clock																					//TODO  Make this populate with defines for switching SPI instances
RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE);
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


//*********************FUNCTIONS************************************
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
  stuff = max7456_transfer_byte(reg | 0x00);
  stuff = max7456_transfer_byte(data);
  max7456_disable();
	return stuff;
}


// Function to read osd registers
uint8_t MAX7456_read(uint8_t reg)
{
	uint8_t stuff;
  max7456_enable();
  max7456_transfer_byte(reg | 0x80);
  stuff = max7456_transfer_byte(0x00);
  max7456_disable();
	return stuff;
}
/*
void max7456_write( uint8_t reg, uint8_t val )
{
  SPI.transfer( reg );
  SPI.transfer( val );
}


uint8_t max7456_read( uint8_t reg )
{ 
  max7456_enable();
  SPI.transfer( reg );
  return SPI.transfer( 255 );
  max7456_disable();
}
*/



void osd_clear()
{
  for ( byte y = 0 ; y < maxrows ; y++)
  {
   osd_print( "          " , 10 ,  0 , y ); 
   osd_print( "          " , 10 ,  10 , y ); 
   osd_print( "          " , 10 ,  20 , y ); 
   }
}



byte floattochar(char * buffer, float val)
{// this did not work the simpler way (with %f ) 
  unsigned int volta;
  unsigned int voltb;// decimal digits
  volta = trunc ( val);
  voltb = round ((float)( val - volta)*100 );
  volta = constrain( volta , 0 , 99);
  voltb = constrain( voltb , 0 , 99); 
  byte cx;
  cx = snprintf ( buffer, 10, "%d.%02d",(int) volta, voltb );

 return cx;
}





void osd_print( char *buffer , uint8_t len,  uint8_t x , uint8_t y)
{
    if( lastsystem == NTSC)
    {
      //NTSC adjustment 3 lines up if after line 8
      //
      if ( y > 7 ) y = y - 3; 
      
    }
  if ( y > 16 ) y = 16;
  
  uint8_t pos = x + y*30;
  
  cs_on();
   // 16 bit mode, auto increment mode 
  spi_write(DMM, B00000001 );
  // memory address
  spi_write(DMAH, B00000001 & (pos >> 8) );
  spi_write(DMAL, (byte) pos  );
  
  for ( int i = 0; i < len; i++ ) {
  spi_write( DMDI, buffer[i] );
  }
  // off autoincrement mode
  spi_write( DMDI, 0xFF );
  cs_off(); 
}




void max7456init()
{
byte x;
  
cs_on();
spi_write( VM0, B00000010 ); // soft reset
cs_off();

delay(1);

x = spi_read(OSDBL_R); // mine set to 31 (factory) 
cs_on();
spi_write( OSDBL_W , x|B00010000 );
cs_off();

cs_on();
if ( osdsystem == PAL )
  {   
  spi_write( VM0, B01001000 ); // Set pal mode ( ntsc by default) and enable display
  lastvm0 = B01001000;
  }
else 
  {
  spi_write( VM0, B00001000); 
  lastvm0 = B00001000;
  }

 spi_write( VM1, B00001110);
 
 spi_write( 0x0C, B00101101); // osd mux rise/fall ( lowest sharpness)
 
 cs_off();

}




void  checksystem()
{
  if ( !systemclear && millis() - timesystem > 3000)
  {
  // delete the PAL /NTSC /NONE message after a timeout
  osd_print( "    " , 4 , SYSTEMXPOS , SYSTEMYPOS ); 
  systemclear = 1;     
  }
  
  // check detected video system
  byte x = spi_read(STAT);
  if (x & B00000001) 
  { //PAL
  //Serial.println("PAL");
  if ( lastsystem != PAL )
    {
     osd_print( "PAL " , 4 , SYSTEMXPOS , SYSTEMYPOS );
     timesystem = millis(); 
     systemclear = 0;
     lastsystem = PAL;
     if ( osdsystem != PAL ) 
       {
         osd_clear();
         osd_setsystem(PAL);
       }
    }
  } 
  if (x & B00000010) 
  { //NTSC
  //Serial.println("NTSC");
   if ( lastsystem != NTSC )
    {
    osd_print( "NTSC" , 4 , SYSTEMXPOS , SYSTEMYPOS );
    timesystem = millis(); 
    systemclear = 0;
    lastsystem = NTSC;
    if ( osdsystem != NTSC ) 
      {
       osd_clear();
       osd_setsystem(NTSC);
      }
    }
  }  
  
   if ( ! ( x|B00000011 ) )
  {//No signal
   //Serial.println("NONE");
   if ( lastsystem != 2 )
    {
    osd_print( "NONE" , 4 , SYSTEMXPOS , SYSTEMYPOS );
    timesystem = millis(); 
    systemclear = 0;
    lastsystem = 2;
    }
    
  }  
 return;
}


// set the video output system PAL /NTSC
void osd_setsystem( byte sys)
{
byte x = spi_read(VM0_R); 

cs_on();
if ( sys == PAL )
    {
     lastvm0 = x| B01000000; 
     spi_write( VM0, x|( B01000000 ) );
    }
else
    {
     lastvm0 =  x& B10111111;
     spi_write( VM0, x&( B10111111 ) );
    }
cs_off();
 
}



void osd_intro()
{
 osd_print( "smallOSD" , 8 ,  5 , 5 );  // row , col
 osd_print( "v0.8" , 4 ,  5 , 6 );  // row , col   
}


// check for osd "accidental" reset
// the MAX7456 and atmega328 have very different voltage ranges
// MAX resets somewhere between 4.2V and 4.6V
// atmega328 works to below 3.3V (depending on brownout fuses)
void check_osd()
{
  byte x = spi_read(VM0_R);  
 
  if ( x != lastvm0)
  {// the register is not what it's supposed to be

   cs_on();
   spi_write( VM0, B00000010 ); // soft reset
   cs_off();
   delay(1);
   // only set minimum number of registers for functionality
   cs_on();    
   if ( osdsystem == PAL )
    {
    spi_write( VM0, B01001000 ); // Set pal mode ( ntsc by default) and enable display
    lastvm0 = B01001000;
    }
   else
    {
    spi_write( VM0, B00001000); // set NTSC
    lastvm0 = B00001000;
    }
   cs_off();
   flag = 1;
  } 
}
#endif



