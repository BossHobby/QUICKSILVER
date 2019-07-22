// ltm protocol 
// This is currently non - standard in that it uses 115200 baudrate
// Some of the data is also adjusted to work with current dev version of MWOSD
// current = june '16
//
// a dummy gps frame is sent to stop "gps alerts"
// the routine sends attitude, fc volts and rssi ( if available)

#include "binary.h"
#include "defines.h"
#include "config.h"
#include "rx_bayang.h" // for struct rxdebug;
#include <stdio.h>


#ifdef OSD_LTM_PROTOCOL

char crc;

//extern int fputc(int ch, FILE * f);

extern void buffer_add(int val );

void Serial_print(char ch)
{
	buffer_add( (int) ch );
}


void sendbyte( char val)
{
  crc ^= val;
  Serial_print((char) val ); 
}

void sendint( int val)
{
  sendbyte( (char) (val) );
  sendbyte( (char) (val>>8) );
}

void sendcrc()
{
Serial_print((char) (crc) ); 
}


void sendheader()
{
 sendbyte((char)'$');
 sendbyte((char)'T');
}

// a frame
// 6 bytes
//    Pitch, int16, [degrees]
//    Roll, int16, [degrees]
//    Heading, int16, [degrees]

extern float attitude[];

void send_a_frame()
{
 sendheader();
 Serial_print((char) 'A');
 crc = 0;
 sendint( attitude[0] + 0.5f );// 
 sendint( attitude[1] + 0.5f); // roll (pitch?)
 sendint(0); //heading
 sendcrc();
}

// g frame
// 14 bytes

//    Latitude, int32 [decimal degrees * 10,000,000]
//    Longitude, int32 [decimal degrees * 10,000,000]
//    Ground Speed, uchar [m/s]
//    Altitude, uint32, [cm]
//    Sats, uchar,
//    bits 0-1 : fix
//    bits 2-7 : number of satellites

void send_g_frame()
{
 sendheader();
 Serial_print( 'G');
 crc = 0;
/*
 sendint( 0 ); // lat
 sendint( 0 ); // lat2
 sendint( 0 ); // long
 sendint( 0 ); // long2 
	*/
 sendbyte(0); // groundspeed (m/s)
// sendint(0);  // alti (cm)
// sendint(0);  // alti2
	
// sending all above dummy data in one go for optimization
for ( int i = 0 ; i < 6; i++)
	{
		 sendint(0);
	}
	
 sendbyte(B00111111); // sats
 sendcrc();
}
// S frame
// 7 bytes

//  Vbat, uint16, [mV]
//  Current, uint16, [mA]
//  RSSI, uchar
//  Airspeed, uchar8, [m/s]
//  Status, uchar

extern float vbattfilt;
extern int failsafe;
extern int rxmode;
extern char aux[];


//extern int packetpersecond;
extern struct rxdebug rxdebug;

void send_s_frame()
{
 sendheader();
 Serial_print('S');
 crc = 0;
 sendint( (unsigned int) vbattfilt *10 + 0.5f );// vbatt mV 126 = 12.6
 sendint( 1000 ); // current mA
	
int rssi = rxdebug.packetpersecond;
if (rssi > 255) rssi = 255;
	
 sendbyte(rssi); // rssi
 sendbyte(0); // airspeed
#define ARMED ( (rxmode!=RXMODE_BIND) )
#define FAILSAFE failsafe
#define MODE ( (aux[LEVELMODE])?3:4 )

//0 : Manual
//1 : Rate
//2 : Angle
//3 : Horizon
//4 : Acro
 char status = (ARMED<<0)|(FAILSAFE<<1)|(MODE<<2);
 
 sendbyte(status); // status
 
 sendcrc();
}


int osdcount = 0;

void osdcycle()
{
	osdcount++;
	if (osdcount%30 == 29)
	{
		send_a_frame();
		return;
	}
	
	if (osdcount%999 == 1)
	{
		send_g_frame();
		return;
	}
	
		if (osdcount%332 == 5)
	{
		send_s_frame();
		return;
	}
	
}
#else
// ods disabled - dummy functions
void osdcycle()
{
	
}




#endif

