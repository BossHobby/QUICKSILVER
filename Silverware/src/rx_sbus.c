// serial for stm32 not used yet
#include "project.h"
#include <stdio.h>
#include "drv_serial.h"
#include "config.h"
#include "drv_time.h"
#include "defines.h"
#include "util.h"
#include "uart.h"


// sbus input ( pin SWCLK after calibration) 
// WILL DISABLE PROGRAMMING AFTER GYRO CALIBRATION - 2 - 3 seconds after powerup)


#ifdef RX_SBUS

#define SERIAL_BAUDRATE 100000

// sbus is normally inverted
#define SBUS_INVERT 1

// global use rx variables
extern float rx[4];
extern char aux[AUXNUMBER];
extern char lastaux[AUXNUMBER];
extern char auxchange[AUXNUMBER];
int failsafe = 0;
int rxmode = 0;
int rx_ready = 0;


// internal sbus variables
#define RX_BUFF_SIZE 64							//SPEK_FRAME_SIZE 16  
uint8_t rx_buffer[RX_BUFF_SIZE];    //spekFrame[SPEK_FRAME_SIZE]
uint8_t rx_start = 0;
uint8_t rx_end = 0;
uint16_t rx_time[RX_BUFF_SIZE];			//????

int framestarted = -1;
uint8_t framestart = 0;


unsigned long time_lastrx;
unsigned long time_siglost;
uint8_t last_rx_end = 0;
int last_byte = 0;
unsigned long time_lastframe;
int frame_received = 0;
int rx_state = 0;
int bind_safety = 0;
uint8_t data[25];
int channels[9];

int failsafe_sbus_failsafe = 0;   
int failsafe_siglost = 0; 
int failsafe_noframes = 0;

// enable statistics
const int sbus_stats = 0;

// statistics
int stat_framestartcount;
int stat_timing_fail;
int stat_garbage;
//int stat_timing[25];
int stat_frames_accepted = 0;
int stat_frames_second;
int stat_overflow;


//void SERIAL_RX_USART_IRQHandler(void)
void sbus_USART_ISR(void)
{
    rx_buffer[rx_end] = USART_ReceiveData(SERIAL_RX_USART);
    // calculate timing since last rx
    unsigned long  maxticks = SysTick->LOAD;	
    unsigned long ticks = SysTick->VAL;	
    unsigned long elapsedticks;	
    static unsigned long lastticks;
    if (ticks < lastticks) 
        elapsedticks = lastticks - ticks;	
    else
        {// overflow ( underflow really)
        elapsedticks = lastticks + ( maxticks - ticks);	
        }

    if ( elapsedticks < 65536 ) rx_time[rx_end] = elapsedticks; //
    else rx_time[rx_end] = 65535;  //ffff

    lastticks = ticks;
       
    if ( USART_GetFlagStatus(SERIAL_RX_USART , USART_FLAG_ORE ) )
    {
      // overflow means something was lost 
      rx_time[rx_end]= 0xFFFe;
      USART_ClearFlag( SERIAL_RX_USART , USART_FLAG_ORE );
      if ( sbus_stats ) stat_overflow++;
    }    
        
    rx_end++;
    rx_end%=(RX_BUFF_SIZE);
}



void sbus_init(void)
{
    // make sure there is some time to program the board
    if ( gettime() < 2000000 ) return;
    
    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = SERIAL_RX_PIN;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure); 
    GPIO_PinAFConfig(SERIAL_RX_PORT, SERIAL_RX_SOURCE , SERIAL_RX_CHANNEL);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SERIAL_RX_USART, ENABLE);

    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = SERIAL_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx ;//USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(SERIAL_RX_USART, &USART_InitStructure);
// swap rx/tx pins
#ifndef Alienwhoop_ZERO
    USART_SWAPPinCmd( SERIAL_RX_USART, ENABLE);
#endif
// invert signal ( default sbus )
   if (SBUS_INVERT) USART_InvPinCmd(SERIAL_RX_USART, USART_InvPin_Rx|USART_InvPin_Tx , ENABLE );


    USART_ITConfig(SERIAL_RX_USART, USART_IT_RXNE, ENABLE);

    USART_Cmd(SERIAL_RX_USART, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_RX_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


    rxmode = !RXMODE_BIND;

// set setup complete flag
 framestarted = 0;
}


void rx_init(void)
{
    
}


void checkrx()
{
 
if ( framestarted == 0)
{
    while (  rx_end != rx_start )
    { 
    if ( rx_buffer[rx_start] == 0x0f )
            {
                // start detected
                framestart = rx_start;
                framestarted = 1;  
                stat_framestartcount++; 
            break;                
            }         
    rx_start++;
    rx_start%=(RX_BUFF_SIZE);
            
    stat_garbage++;
    }
            
}
else if ( framestarted == 1)
{
 // frame already has begun
 int size = 0;
    if (rx_end > framestart ) size = rx_end - framestart;
    else size = RX_BUFF_SIZE - framestart + rx_end;
 if ( size >= 24 )
    {    
    int timing_fail = 0; 
        
    for ( int i = framestart ; i <framestart + 25; i++  )  
    {
      data[ i - framestart] = rx_buffer[i%(RX_BUFF_SIZE)];
      int symboltime = rx_time[i%(RX_BUFF_SIZE)];
      //stat_timing[ i - framestart] = symboltime;
      if ( symboltime > 1024 &&  i - framestart > 0 ) timing_fail = 1;
    }    

   if (!timing_fail) 
   {
       frame_received = 1;
          
      if (data[23] & (1<<2)) 
      {       
       // frame lost bit
       if ( !time_siglost ) time_siglost = gettime();
       if ( gettime() - time_siglost > 1000000 ) 
       {
           failsafe_siglost = 1;   
       }
      }
      else
      {
        time_siglost = 0;  
        failsafe_siglost = 0;
      }

      
      if (data[23] & (1<<3)) 
      {
        // failsafe bit
        failsafe_sbus_failsafe = 1;
      }
      else{
          failsafe_sbus_failsafe = 0;
      }
      
           
   }else if (sbus_stats) stat_timing_fail++; 
    
   last_byte = data[24];

  
    rx_start = rx_end;
    framestarted = 0;
    bind_safety++;
    } // end frame complete  
    
}// end frame pending
else
    if ( framestarted < 0)
    {
        // initialize sbus
      sbus_init();
       // set in routine above "framestarted = 0;"    
    }
      
if ( frame_received )
{
   int channels[9];
   //decode frame    
   channels[0]  = ((data[1]|data[2]<< 8) & 0x07FF);
   channels[1]  = ((data[2]>>3|data[3]<<5) & 0x07FF);
   channels[2]  = ((data[3]>>6|data[4]<<2|data[5]<<10) & 0x07FF);
   channels[3]  = ((data[5]>>1|data[6]<<7) & 0x07FF);
   channels[4]  = ((data[6]>>4|data[7]<<4) & 0x07FF);
   channels[5]  = ((data[7]>>7|data[8]<<1|data[9]<<9) & 0x07FF);
   channels[6]  = ((data[9]>>2|data[10]<<6) & 0x07FF);
   channels[7]  = ((data[10]>>5|data[11]<<3) & 0x07FF);
   channels[8]  = ((data[12]|data[13]<< 8) & 0x07FF);

    if ( rx_state == 0)
    {
     // wait for valid sbus signal
     static int frame_count = 0; 
     failsafe = 1;
     rxmode = RXMODE_BIND; 
     // if throttle < 10%   
     if (  channels[2] < 336 ) frame_count++;
     if (frame_count  > 130 )
     {
         if( stat_frames_second > 30 )
         {
             rx_state++; 
             rxmode = !RXMODE_BIND; 
         }
         else
         {
             frame_count = 0;
         }
     }
      
    }
    
    if ( rx_state == 1)
    {
      // normal rx mode
        
      // AETR channel order
        channels[0] -= 993;           
        channels[1] -= 993;
        channels[3] -= 993;      
        
        rx[0] = channels[0];  
        rx[1] = channels[1]; 
        rx[2] = channels[3];  
      
        for ( int i = 0 ; i < 3 ; i++)
        {
         rx[i] *= 0.00122026f;             
        }
        
        channels[2]-= 173; 
        rx[3] = 0.000610128f * channels[2]; 
        
        if ( rx[3] > 1 ) rx[3] = 1;
				
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
							
			aux[CHAN_5] = (channels[4] > 993) ? 1 : 0;
		    aux[CHAN_6] = (channels[5] > 993) ? 1 : 0;
		    aux[CHAN_7] = (channels[6] > 993) ? 1 : 0;
		    aux[CHAN_8] = (channels[7] > 993) ? 1 : 0;
			aux[CHAN_9] = (channels[8] > 993) ? 1 : 0;
        
        time_lastframe = gettime(); 
        if (sbus_stats) stat_frames_accepted++;
				if (bind_safety > 9){								//requires 10 good frames to come in before rx_ready safety can be toggled to 1
				rx_ready = 1;											// because aux channels initialize low and clear the binding while armed flag before aux updates high
				bind_safety = 10;}								
    }
 

// stats
    static int fps = 0;
    static unsigned long secondtime = 0;
    
    if ( gettime() - secondtime > 1000000 )
    {
       stat_frames_second = fps;
       fps = 0;
       secondtime = gettime();
    }
    fps++;
    
frame_received = 0;    
} // end frame received


if ( gettime() - time_lastframe > 1000000 )
{
    failsafe_noframes = 1;
}else failsafe_noframes = 0;

// add the 3 failsafes together
    failsafe = failsafe_noframes || failsafe_siglost || failsafe_sbus_failsafe;

}

#endif



