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


// radio settings

// packet period in uS
#define PACKET_PERIOD 2000
#define PACKET_PERIOD_TELEMETRY 5000

// was 250 ( uS )
#define PACKET_OFFSET 000

#ifdef USE_STOCK_TX
#undef PACKET_PERIOD
#define PACKET_PERIOD 2000
#undef PACKET_OFFSET
#define PACKET_OFFSET 0
#endif

// how many times to hop ahead if no reception
#define HOPPING_NUMBER 4

//#define RX_DATARATE_250K

//
#define XN_TO_TX B00000010
#define XN_TO_RX B00000011

#define RX_MODE_NORMAL RXMODE_NORMAL
#define RX_MODE_BIND RXMODE_BIND


#ifdef RX_NRF24_BAYANG_TELEMETRY

// crc enable - rx side
#define crc_en 1 // zero or one only

void writeregs(uint8_t data[], uint8_t size)
{
    spi_cson();
    for (uint8_t i = 0; i < size; i++)
      {
          spi_sendbyte(data[i]);
      }
    spi_csoff();
}

// nrf297 to nrf24 emulation based on code from 
// nrf24multipro by goebish
// deviationTx by various contribuitors

const uint8_t xn297_scramble[] = {
    0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
    0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
    0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc };

   
    
// from https://graphics.stanford.edu/~seander/bithacks.html#ReverseByteWith32Bits
// reverse the bit order in a single byte
uint8_t swapbits(uint8_t a){
unsigned int b = a;
b = ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
return b;
}


uint16_t crc16_update(uint16_t crc, uint8_t in)
{
    crc ^= in << 8;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
            } else {
            crc = crc << 1;
        }
    }
    return crc;
}
 
// crc calculated over address field ( constant)
uint16_t crc_addr = 0;

// set both rx and tx address to a xn297 address
// the tx address can only send to another nrf24 
// because it lacks the xn297 preamble
void nrf24_set_xn297_address( uint8_t* addr )
{
    uint8_t rxaddr[6] = {0x2a,  };
    crc_addr = 0xb5d2;
    for (int i = 5; i > 0; i--) {
        rxaddr[i] = addr[i - 1] ^ xn297_scramble[5-i];
        if ( crc_en ) crc_addr = crc16_update(crc_addr, rxaddr[i]);
    }

    // write rx address
    writeregs( rxaddr , sizeof(rxaddr) );
    // write tx address
    rxaddr[0] = 0x30;
    writeregs( rxaddr , sizeof(rxaddr) );
}



int crc_error = 0;

int nrf24_read_xn297_payload(int * rxdata, int size)
{
    // 81uS
    xn_readpayload(rxdata, size);

    if ( crc_en )
    {
    // 65uS
    uint16_t crcx;
    crcx = crc_addr;    
    for (uint8_t i = 0; i < size-2; i++) {
        crcx = crc16_update(crcx, rxdata[i]);
    }
    uint16_t crcrx =  rxdata[size-2]<<8;
     crcrx |=  rxdata[size-1]&0xFF;
    
    // hardcoded for len 15
    if ( (crcx^crcrx) != 0x9BA7)
    {
        crc_error++;
        return 0;
    }
    }
    //29uS
    for(int i=0 ; i<size- crc_en*2 ; i++)
    {
      rxdata[i] = swapbits(rxdata[i] ^ xn297_scramble[i+5]);        
    }
    
// crc correct or not used 
return 1;
}


void nrf24_write_xn297_payload(int * txdata, int size)
{    
    for(int i=0 ; i<size ; i++)
    {
      txdata[i] = swapbits(txdata[i]) ^ xn297_scramble[i+5];        
    }
     
  xn_writepayload(txdata, size);
}





extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];


char lasttrim[4];
char rfchannel[4];
int rxaddress[5];
int rxmode = 0;
int rf_chan = 0;
int rx_ready = 0;
int bind_safety = 0;
int rxdata[17 + 2* crc_en];




void rx_init()
{


// always on (CH_ON) channel set 1
    aux[AUXNUMBER - 2] = 1;
// always off (CH_OFF) channel set 0
    aux[AUXNUMBER - 1] = 0;
#ifdef AUX1_START_ON
    aux[CH_AUX1] = 1;
#endif


    
#ifdef RADIO_CHECK
    int rxcheck = xn_readreg(0x0f); // rx address pipe 5   
    // should be 0xc6
    extern void failloop(int);
    if (rxcheck != 0xc6)
        failloop(3);
#endif 

    delay(100);
        
   static uint8_t rxaddr[5] = {  0 , 0 , 0 , 0 , 0  };
   nrf24_set_xn297_address( rxaddr );

    
    xn_writereg(EN_AA, 0);      // aa disabled
    xn_writereg(EN_RXADDR, 1);  // pipe 0 only
   #ifdef RX_DATARATE_250K
    xn_writereg(RF_SETUP, B00100110);     // power / data rate 250K
   #else
    xn_writereg(RF_SETUP, B00000110);    // power / data rate 1000K
   #endif

    xn_writereg(RX_PW_P0, 15+ crc_en *2);  // payload size
    xn_writereg(SETUP_RETR, 0); // no retransmissions
    xn_writereg(SETUP_AW, 3);   // address size (5 bytes)
    xn_writereg(RF_CH, 0);      // bind on channel 0
    xn_command(FLUSH_RX);
    xn_writereg(0, XN_TO_RX);   // power up, crc disabled, rx mode

}



//#define RXDEBUG

#ifdef RXDEBUG
unsigned long packettime;
int channelcount[4];
int failcount;
int skipstats[12];
int afterskip[12];

#warning "RX debug enabled"
#endif

int packetrx;
int packetpersecond;


void send_telemetry(void);
void nextchannel(void);

int loopcounter = 0;
unsigned int send_time;
int telemetry_send = 0;
int oldchan = 0;

#define TELEMETRY_TIMEOUT 10000

void beacon_sequence()
{
    static int beacon_seq_state = 0;

    switch (beacon_seq_state)
      {
          case 0:
              // send data
              telemetry_send = 1;
              send_telemetry();
              beacon_seq_state++;
              break;

          case 1:
              // wait for data to finish transmitting
              if ((xn_readreg(0x17) & B00010000))
                {
                    xn_writereg(0, XN_TO_RX);
                    beacon_seq_state = 0;
                    telemetry_send = 0;
                    nextchannel();
                }
              else
                {   // if it takes too long we get rid of it
                    if (gettime() - send_time > TELEMETRY_TIMEOUT)
                      {
                          xn_command(FLUSH_TX);
                          xn_writereg(0, XN_TO_RX);
                          beacon_seq_state = 0;
                          telemetry_send = 0;
                      }
                }
              break;

          default:
              beacon_seq_state = 0;
              break;



      }

}


extern int lowbatt;
extern float vbattfilt;
extern float vbatt_comp;

void send_telemetry()
{

    int txdata[15];
    for (int i = 0; i < 15; i++)
        txdata[i] = i;
    txdata[0] = 133;
    txdata[1] = lowbatt;

    int vbatt = vbattfilt * 100;
// battery volt filtered    
    txdata[3] = (vbatt >> 8) & 0xff;
    txdata[4] = vbatt & 0xff;

    vbatt = vbatt_comp * 100;
// battery volt compensated 
    txdata[5] = (vbatt >> 8) & 0xff;
    txdata[6] = vbatt & 0xff;

    int temp = packetpersecond / 2;
    if (temp > 255)
        temp = 255;

    txdata[7] = temp;           // rx strenght

    if (lowbatt)
        txdata[3] |= (1 << 3);

    int sum = 0;
    for (int i = 0; i < 14; i++)
      {
          sum += txdata[i];
      }

    txdata[14] = sum;
   // xn_command(FLUSH_TX);

    xn_writereg(0, XN_TO_TX);

    nrf24_write_xn297_payload( txdata , 15);
    xn_writereg(0, 0);
    send_time = gettime(); 
    xn_writereg(0, XN_TO_TX); 
 
     
    return;
}



static char checkpacket()
{
    int status = xn_readreg(7);

    if (status & (1 << MASK_RX_DR))
      {                         // rx clear bit

         // xn_writereg( STATUS , (1<<MASK_RX_DR) );
          //RX packet received
         // return 1;
      }
    if ((status & B00001110) != B00001110)
      {
          // rx fifo not empty        
          return 2;
      }

    return 0;
}




float packettodata(int *data)
{
    return (((data[0] & 0x0003) * 256 + data[1]) - 512) * 0.001953125;
}


static int decodepacket(void)
{
    if ( rxdata[0] == 165)
      {
          int sum = 0;
          for (int i = 0; i < 14; i++)
            {
                sum += rxdata[i];
            }
          if ((sum & 0xFF) == rxdata[14])
            {
                rx[0] = packettodata(&rxdata[4]);
                rx[1] = packettodata(&rxdata[6]);
                rx[2] = packettodata(&rxdata[10]);
                // throttle     
                rx[3] =
                    ((rxdata[8] & 0x0003) * 256 +
                     rxdata[9]) * 0.000976562f;



#ifdef USE_STOCK_TX
                char trims[4];
                trims[0] = rxdata[6] >> 2;
                trims[1] = rxdata[4] >> 2;

                for (int i = 0; i < 2; i++)
                    if (trims[i] != lasttrim[i])
                      {
                          aux[CH_PIT_TRIM + i] = trims[i] > lasttrim[i];
                          lasttrim[i] = trims[i];
                      }
#else
                aux[CH_INV] = (rxdata[3] & 0x80) ? 1 : 0;   // inverted flag

                aux[CH_VID] = (rxdata[2] & 0x10) ? 1 : 0;

                aux[CH_PIC] = (rxdata[2] & 0x20) ? 1 : 0;
#endif

                aux[CH_TO] = (rxdata[3] & 0x20) ? 1 : 0;   // take off flag
                      
                aux[CH_EMG] = (rxdata[3] & 0x04) ? 1 : 0;   // emg stop flag
                      
                aux[CH_FLIP] = (rxdata[2] & 0x08) ? 1 : 0;

                aux[CH_EXPERT] = (rxdata[1] == 0xfa) ? 1 : 0;

                aux[CH_HEADFREE] = (rxdata[2] & 0x02) ? 1 : 0;

                aux[CH_RTH] = (rxdata[2] & 0x01) ? 1 : 0;   // rth channel



							if (aux[LEVELMODE]){
								if (aux[RACEMODE] && !aux[HORIZON]){
									if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
									if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
									if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
								}else if (aux[HORIZON]){
									if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
									if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
									if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);
								}else{
									if ( ANGLE_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ANGLE_EXPO_ROLL);
									if ( ANGLE_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ANGLE_EXPO_PITCH);
									if ( ANGLE_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ANGLE_EXPO_YAW);}
							}else{
								if ( ACRO_EXPO_ROLL > 0.01) rx[0] = rcexpo(rx[0], ACRO_EXPO_ROLL);
								if ( ACRO_EXPO_PITCH > 0.01) rx[1] = rcexpo(rx[1], ACRO_EXPO_PITCH);
								if ( ACRO_EXPO_YAW > 0.01) rx[2] = rcexpo(rx[2], ACRO_EXPO_YAW);
							}



                for (int i = 0; i < AUXNUMBER - 2; i++)
                  {
                      auxchange[i] = 0;
                      if (lastaux[i] != aux[i])
                          auxchange[i] = 1;
                      lastaux[i] = aux[i];
                  }

                return 1;       // valid packet 
            }
          return 0;             // sum fail
      }
    return 0;                   // first byte different
}



void nextchannel()
{
    rf_chan++;
    rf_chan &= 3; // same as %4
    xn_writereg(0x25, rfchannel[rf_chan]);
}


unsigned long lastrxtime;
unsigned long failsafetime;
unsigned long secondtimer;

int failsafe = 0;


unsigned int skipchannel = 0;
int lastrxchan;
int timingfail = 0;
int telemetry_enabled = 0;
int packet_period = PACKET_PERIOD;

uint8_t rxaddr[5];
int packets = 0;

void checkrx(void)
{
    int packetreceived = checkpacket();

    if (packetreceived)
      {
          if (rxmode == RX_MODE_BIND)
            {                   
                // rx startup , bind mode
                
                if ( nrf24_read_xn297_payload(rxdata, 15 + 2* crc_en) )  ;
                else return;

                if (rxdata[0] == 0xa4 || rxdata[0] == 0xa3)
                  {   // bind packet
                      if (rxdata[0] == 0xa3)
                        {
                            telemetry_enabled = 1;
                            packet_period = PACKET_PERIOD_TELEMETRY;
                        }
                        
                      rfchannel[0] = rxdata[6];
                      rfchannel[1] = rxdata[7];
                      rfchannel[2] = rxdata[8];
                      rfchannel[3] = rxdata[9];
                        
                       
                      
                      for ( int i = 0 ; i < 5; i++)
                      {
                        rxaddr[i] = rxdata[i+1];
                      }
                      // write new rx and tx address
                      nrf24_set_xn297_address( rxaddr );
                      
                      xn_writereg(0x25, rfchannel[rf_chan]);    // Set channel frequency 
                      
                      rxmode = RX_MODE_NORMAL;

                  }
            }
          else
            {                   // normal mode  
#ifdef RXDEBUG
                channelcount[rf_chan]++;
                packettime = gettime() - lastrxtime;

                if (skipchannel && !timingfail)
                    afterskip[skipchannel]++;
                if (timingfail)
                    afterskip[0]++;

#endif

                unsigned long temptime = gettime();

                int pass = nrf24_read_xn297_payload(rxdata, 15+ 2* crc_en);
                if ( pass ) pass = decodepacket();

                if (pass)
                  {
                      packetrx++;

                      if (telemetry_enabled )
                      {
                          beacon_sequence();
                          
                      }
                      
                      skipchannel = 0;
                      timingfail = 0;
                      lastrxchan = rf_chan;
                      lastrxtime = temptime;
                      failsafetime = temptime;
                      failsafe = 0;
                      if (!telemetry_send)
                          nextchannel();
                  }
                else
                  {
#ifdef RXDEBUG
                      failcount++;
#endif
                  }

            } // end normal rx mode
				bind_safety++;
				if (bind_safety > 9){								//requires 10 good frames to come in before rx_ready safety can be toggled to 1
				rx_ready = 1;											// because aux channels initialize low and clear the binding while armed flag before aux updates high
				bind_safety = 10;}	
      } // end packet received

// finish sending if already started
    if (telemetry_send)
        beacon_sequence();

    unsigned long time = gettime();


    if (time - lastrxtime > (HOPPING_NUMBER * packet_period + 1000)
        && rxmode != RX_MODE_BIND)
      {
          //  channel with no reception   
          lastrxtime = time;
          // set channel to last with reception
          if (!timingfail)
              rf_chan = lastrxchan;
          // advance to next channel
          nextchannel();
          // set flag to discard packet timing
          timingfail = 1;
      }

    if (!timingfail && !telemetry_send && skipchannel < HOPPING_NUMBER + 1
        && rxmode != RX_MODE_BIND)
      {
          unsigned int temp = time - lastrxtime;

          if (temp > 1000
              && (temp - (PACKET_OFFSET)) / ((int) packet_period) >=
              (skipchannel + 1))
            {
                nextchannel();
#ifdef RXDEBUG
                skipstats[skipchannel]++;
#endif
                skipchannel++;
            }
      }

    if (time - failsafetime > FAILSAFETIME)
      {                         //  failsafe
          failsafe = 1;
          rx[0] = 0;
          rx[1] = 0;
          rx[2] = 0;
          rx[3] = 0;
      }

    if (gettime() - secondtimer > 1000000)
      {
          packetpersecond = packetrx;
          packetrx = 0;
          secondtimer = gettime();
      }


}


#endif
