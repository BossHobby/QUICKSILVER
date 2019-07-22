/*
The MIT License (MIT)

Copyright (c) 2016 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


// cx10 protocol is work in progress

#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"
#include "drv_time.h"
#include <stdio.h>
#include "config.h"
#include "defines.h"

#include "rx_bayang.h"

#include "util.h"

#ifdef RX_CX10BLUE_PROTOCOL

#define PAYLOAD_LENGHT 19

extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];

int rxmode = 0;
int failsafe = 0;

void writeregs (  const uint8_t data[] , uint8_t size )
{
spi_cson();
for ( uint8_t i = 0 ; i < size ; i++)
{
	spi_sendbyte( data[i]);
}
spi_csoff();
delay(1000);
}


 uint8_t bbcal[6] = { 0x3f , 0x4c , 0x84 , 0x6F , 0x9c , 0x20  };
 uint8_t rfcal[8] = { 0x3e , 0xc9 , 220 , 0x80 , 0x61 , 0xbb , 0xab , 0x9c  };
 uint8_t demodcal[6] = { 0x39 , 0x0b , 0xdf , 0xc4 , 0xa7 , 0x03};

void rx_init()
{	
/*
writeregs( bbcal , sizeof(bbcal) );
writeregs( rfcal , sizeof(rfcal) );
writeregs( demodcal , sizeof(demodcal) );
*/
int rxaddress[5] =  {0xCC,0xCC,0xCC,0xCC,0xCC};

xn_writerxaddress( rxaddress);

xn_writetxaddress( rxaddress);	

	xn_writereg( EN_AA , 0 );	// aa disabled
	xn_writereg( EN_RXADDR , 1 ); // pipe 0 only
//	xn_writereg( RF_SETUP , B00000001);  // lna high current on ( better performance )
	xn_writereg( RF_SETUP , B00000111);
	xn_writereg( RX_PW_P0 , PAYLOAD_LENGHT ); // payload size
	xn_writereg( SETUP_RETR , 0 ); // no retransmissions ( redundant?)
	xn_writereg( SETUP_AW , 3 ); // address size (5 bits)
	xn_command( FLUSH_RX);
  xn_writereg( RF_CH , 2 );  // bind  channel 
  xn_writereg( 0 , B00001111 ); // power up, crc enabled


	
}


static char checkpacket()
{
int status = xn_readreg( 7 );
  
	if ( status&(1<<MASK_RX_DR) )
	{	 // rx clear bit
		// this is not working well
	 // xn_writereg( STATUS , (1<<MASK_RX_DR) );
		//RX packet received
		//return 1;
	}
	if( (status & B00001110) != B00001110 )
	{
		// rx fifo not empty		
		return 2;	
	}
	
  return 0;
}


int rxdata[PAYLOAD_LENGHT];


float cx10scale( int num)
{
	return (float) (( rxdata[num] + 256*rxdata[num+1] ) - 1500 )*0.002f;
}

static int decodepacket( void)
{
	if ( rxdata[0] == 0x55 )
	{
		rx[0] = -cx10scale(9); // aileron
		rx[1] = -cx10scale(11) ; // elev
		rx[3] = (cx10scale(13) + 1.0f)*0.5f ; // throttle
		rx[2] = cx10scale(15) ; // throttle
				
#ifndef DISABLE_EXPO
							if (aux[LEVELMODE]){
								if (aux[RACEMODE]){
									rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
									rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
									rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
								}else{
									rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
									rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
									rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);}
							}else{
								rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
								rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
								rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
							}
#endif

    aux[0] = (rxdata[16] & 0x10)?1:0;
			
	  aux[2] = (rxdata[17] & 0x01)?1:0; // rates mid
		
		for ( int i = 0 ; i < AUXNUMBER - 2 ; i++)
		{
			auxchange[i] = 0;
			if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
			lastaux[i] = aux[i];
		}
		
		return 1;	// valid packet	
		}
	 return 0; // 
}


int rfchannel[4];
int chan = 0;

unsigned long lastrxtime;
unsigned long failsafetime;



void nextchannel()
{
	chan++;
	if (chan > 3 ) chan = 0;
	xn_writereg(0x25, rfchannel[chan] );
}




#ifdef RXDEBUG	
struct rxdebug
	{
	unsigned long packettime;
	int failcount;
	int packetpersecond;
	int channelcount[4];
	} 
	rxdebug;
int packetrx;
unsigned long lastrxtime;
unsigned long secondtimer;
#warning "RX debug enabled"

#endif


void checkrx( void)
{
	int packetreceived =	checkpacket();
	int pass = 0;
		if ( packetreceived ) 
		{ 
			if ( rxmode == 0)
			{	// rx startup , bind mode
				xn_readpayload( rxdata , 15);
		
				if ( rxdata[0] == 0xAA ) 
				{// bind packet
					
				  unsigned int temp = rxdata[2];//&0x2F;	
					
					rfchannel[0] = ( (uint8_t) rxdata[1] & 0x0F) + 0x03;
					rfchannel[1] = ( (uint8_t) rxdata[1] >> 4) + 0x16;
					rfchannel[2] = ( (uint8_t) temp & 0x0F) + 0x2D;
					rfchannel[3] = ( (uint8_t) temp >> 4);
					
					rxdata[9] = 1;
					for ( int i = 200; i!=0; i--)
					{
						// sent confirmation to tx  
																
					xn_writereg( 0 , B00001110 ); 
					delay(130);
					
					xn_writepayload(  rxdata , PAYLOAD_LENGHT );
					/*					
					int status;
					status = 0;
					int txcount = 0;
					while( !(status&B00100000) && txcount < 0x100 ) 
					{
						status = xn_command(NOP);
						delay(10);
						txcount++;
					}
					*/
					delay(1000);
					xn_writereg( 0 , B00001111 ); 
					//xn_writereg( STATUS , B00100000 );
					delay(1000);
					}
					rxmode = RXMODE_NORMAL;				
					
					nextchannel();
				  extern unsigned long lastlooptime;
					lastlooptime = gettime();
					#ifdef SERIAL	
					printf( " BIND \n");
					#endif
				}
			}
			else
			{	// normal rx mode	
				#ifdef RXDEBUG	
				rxdebug.packettime = gettime() - lastrxtime;
				#endif
		
				
				lastrxtime = gettime();				
				xn_readpayload( rxdata , PAYLOAD_LENGHT);
				pass = decodepacket();
				 
				if (pass)
				{ 
					
					#ifdef RXDEBUG	
					packetrx++;
					rxdebug.channelcount[chan]++;	
					#endif
					failsafetime = lastrxtime; 
					failsafe = 0;			
					nextchannel();
				
				}	
				else
				{
				#ifdef RXDEBUG	
				rxdebug.failcount++;
				#endif	
				}
			
			}// end normal rx mode
				
		}// end packet received

		unsigned long time = gettime();
		
		if( time - lastrxtime > 20000 && rxmode != RXMODE_BIND)
		{//  channel with no reception	 
		 lastrxtime = time;
		 nextchannel();
		
		}
		if( time - failsafetime > FAILSAFETIME )
		{//  failsafe
		  failsafe = 1;
			rx[0] = 0;
			rx[1] = 0;
			rx[2] = 0;
			rx[3] = 0;
		}
#ifdef RXDEBUG	
			if ( gettime() - secondtimer  > 1000000)
			{
				rxdebug.packetpersecond = packetrx;
				packetrx = 0;
				secondtimer = gettime();
			}
#endif

}

// end bayang protocol
#endif 







