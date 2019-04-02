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

#ifdef RX_BAYANG_TEMPLATE

// the xn297 is not directly compatible with nrf24,
// but an "emulation layer" was made which can communicate with it

// the rf channels are the same, 
// and main registers are also the same with some small exceptions
// some "debug register" allow configuration of some advanced features

// the xn297 does not have 250Kbps, the L version has
// debug registers are not the same between them

// A quick rf description:
// xn297 waits for a custom preamble before the normal data described in nrf24 protocols
// it also scrambles payloads and has a reverse bit order
// xn297 is actually closer to BLE then it is to nrf24


// RPY and Throttle
// range -1.0f to 1.0f 1.0 = full rate as configured
// throttle range 0.0f to 1.0f = under 0.1f it's interpreted as off
extern float rx[4];
// digital on / off channels
extern char aux[AUXNUMBER];
// last value of above
extern char lastaux[AUXNUMBER];
// 1 if change in aux from last value
extern char auxchange[AUXNUMBER];

int failsafe = 0;

void writeregs ( uint8_t data[] , uint8_t size )
{
spi_cson();
for ( uint8_t i = 0 ; i < size ; i++)
{
	spi_sendbyte( data[i]);
}
spi_csoff();
delay(1000);
}


#ifdef RADIO_XN297L
// the xn297l works with default register values ok

// tx power 0-7
#ifndef TX_POWER
#define TX_POWER 7
#endif
	
// Gauss filter amplitude - lowest to fix telemetry issue
//static uint8_t demodcal[2] = { 0x39 , B00000001 };
//writeregs( demodcal , sizeof(demodcal) );

// powerup defaults
//static uint8_t rfcal2[7] = { 0x3a , 0x45 , 0x21 , 0xef , 0xac , 0x3a , 0x50};
//writeregs( rfcal2 , sizeof(rfcal2) );
	
#define XN_TO_RX B10001111
#define XN_TO_TX B10000010
#define XN_POWER B00000111|((TX_POWER&7)<<3)

#endif



#ifdef RADIO_XN297
// xn297 ( without L ) needs register config to receive data

    static uint8_t bbcal[6] = { 0x3f, 0x4c, 0x84, 0x6F, 0x9c, 0x20 };
    writeregs(bbcal, sizeof(bbcal));

    static uint8_t rfcal[8] = { 0x3e, 0xc9, 0x9a, 0xA0, 0x61, 0xbb, 0xab, 0x9c };
    writeregs(rfcal, sizeof(rfcal));

    static uint8_t demodcal[6] = { 0x39, 0x0b, 0xdf, 0xc4, B00100111, B00000000 };
    writeregs(demodcal, sizeof(demodcal));

// tx power 0-3
#ifndef TX_POWER
#define TX_POWER 3
#endif


#define XN_TO_RX B00001111
#define XN_TO_TX B00000010
#define XN_POWER (B00000001|((TX_POWER&3)<<1))
#endif

void rx_init()
{

    delay(100);

// write rx address " 0 0 0 0 0 "        
   static uint8_t rxaddr[6] = { 0x2a , 0 , 0 , 0 , 0 , 0  };
   writeregs( rxaddr , sizeof(rxaddr) );
 
    xn_writereg(RF_CH, 0);      // set radio to channel 0 for bind
   
   
    xn_writereg(EN_AA, 0);      // aa disabled
    xn_writereg(EN_RXADDR, 1);  // pipe 0 only
    xn_writereg(RF_SETUP, XN_POWER);    // power / data rate / lna
    xn_writereg(RX_PW_P0, 15);  // payload size
    xn_writereg(SETUP_RETR, 0); // no retransmissions ( redundant?)
    xn_writereg(SETUP_AW, 3);   // address size (5 bytes)
    xn_command(FLUSH_RX);
   
   

#ifdef RADIO_XN297L
   // it should work without as well
    xn_writereg(0x1d, B00111000);   // 64 bit payload , software ce
    spi_cson();
    spi_sendbyte(0xFD);         // internal CE high command
    spi_sendbyte(0);            // required for above
    spi_csoff();
#endif

#ifdef RADIO_XN297
    xn_writereg(0x1d, B00011000);   // 64 bit payload , software ce
#endif

    xn_writereg(0, XN_TO_RX);   // power up, crc enabled, rx mode

#ifdef RADIO_CHECK
    int rxcheck = xn_readreg(0x0f); // rx address pipe 5   
    // should be 0xc6
    extern void failloop(int);
    if (rxcheck != 0xc6)
        failloop(3);
#endif
	
}


// check if a packet ( or several) has been received
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

int rxdata[15];

float packettodata(int *data)
{
    return (((data[0] & 0x0003) * 256 + data[1]) - 512) * 0.001953125;
}


// decode the receive data and set variables accordingly
static int decodepacket( void)
{
    // if header is correct for a data packet
	if ( rxdata[0] == 165 )
	{
        // calculate checksum
		 int sum = 0;
		 for(int i=0; i<14; i++) 
		 {
			sum += rxdata[i];
		 }	
		if ( (sum&0xFF) == rxdata[14] )
		{
			rx[0] = packettodata( &rxdata[4] );
			rx[1] = packettodata( &rxdata[6] );
			rx[2] = packettodata( &rxdata[10] );
		// throttle		
			rx[3] = ( (rxdata[8]&0x0003) * 256 + rxdata[9] ) * 0.000976562;
		
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


				aux[CH_INV] = (rxdata[3] & 0x80)? 1 : 0; // inverted flag
						
				aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;
												
				aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;						

							
			    aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

			    aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

			    aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

			    aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0;	// rth channel


			for ( int i = 0 ; i < AUXNUMBER - 2 ; i++)
			{
				auxchange[i] = 0;
				if ( lastaux[i] != aux[i] ) auxchange[i] = 1;
				lastaux[i] = aux[i];
			}
			
			return 1;	// valid packet	
		}
	 return 0; // sum fail
	}
return 0; // first byte different
}


  char rfchannel[4];
	int rxaddress[5];
	int rxmode = 0;
	int chan = 0;

void nextchannel()
{
	chan++;
	if (chan > 3 ) chan = 0;
	xn_writereg(0x25, rfchannel[chan] );
}


unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;


// debug variables

unsigned long packettime;
int channelcount[4];
int failcount;
int packetrx;
int packetpersecond;



int timingfail = 0;

// packet period in uS
#define PACKET_PERIOD 3000


void checkrx(void)
{
	int packetreceived = checkpacket();
	int pass = 0;
	if (packetreceived)
	  {
		  if (rxmode == RXMODE_BIND)
		    {	
                // rx bind mode , packet received
			    xn_readpayload(rxdata, 15);

			    if (rxdata[0] == 164)
			      {	// bind packet
				      rfchannel[0] = rxdata[6];
				      rfchannel[1] = rxdata[7];
				      rfchannel[2] = rxdata[8];
				      rfchannel[3] = rxdata[9];
							
					int rxaddress[5];
				      rxaddress[0] = rxdata[1];
				      rxaddress[1] = rxdata[2];
				      rxaddress[2] = rxdata[3];
				      rxaddress[3] = rxdata[4];
				      rxaddress[4] = rxdata[5];
				      //write new address for data packets
				      xn_writerxaddress(rxaddress);
                      // Set channel frequency for data packet
				      xn_writereg(0x25, rfchannel[chan]);	
                      
                    // set mode to data packets  
					rxmode = RXMODE_NORMAL;

			      }
		    }
		  else
		    {		
                // normal mode  

                unsigned long temptime = gettime();
	
			    nextchannel();
                //read payload
			    xn_readpayload(rxdata, 15);
			    pass = decodepacket();
                
                // check if packet is valid
			    if (pass)
			      {   
                      lastrxtime = temptime;
				      failsafetime = temptime;
                      // reset failsafe flag
				      failsafe = 0;
                      //statistics
				      packetrx++;
					
			      }
			    else
			      {
				      failcount++;
			      }

		    }// end normal rx mode

	  }// end packet received

		
	unsigned long time = gettime();

		
	// sequence period 12000
    // if nothing received for a while, we change channel
	if (time - lastrxtime > (PACKET_PERIOD * 3) && rxmode != RXMODE_BIND)
	  {			
			//  channel with no reception   
		  lastrxtime = time;
			
			// advance to next channel
		  nextchannel();

	  }
			
	// failsafe check
	if (time - failsafetime > FAILSAFETIME)
	  {        
          // set failsafe flag
		  failsafe = 1;
          
          // set sticks to zero
		  rx[0] = 0;
		  rx[1] = 0;
		  rx[2] = 0;
		  rx[3] = 0; // throttle zero just in case
          // failsafe flag should cut throttle anyway
	  }
      
// calculate packet rate for debugging
	if (gettime() - secondtimer > 1000000)
	  {
		  packetpersecond = packetrx;
		  packetrx = 0;
		  secondtimer = gettime();
	  }


}

#endif







