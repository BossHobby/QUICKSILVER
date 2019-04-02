#include "gestures.h"
#include "sixaxis.h"
#include "drv_time.h"
#include "defines.h"
#include "config.h"
#include "pid.h"


extern int ledcommand;
extern int ledblink;
extern int onground;
extern char aux[AUXNUMBER];

int pid_gestures_used = 0;

void gestures( void)
{
	#ifndef DISABLE_GESTURES2
		int command = gestures2();

		if (command!=GESTURE_NONE)
        {
            if (command == GESTURE_DDD)
		    { 
			                  
                //skip accel calibration if pid gestures used
                if ( !pid_gestures_used )
                { 
                    gyro_cal();	// for flashing lights
                    acc_cal();                   
                }
                else
                {
                    ledcommand = 1;
                    pid_gestures_used = 0;
                }
                #ifdef FLASH_SAVE2
                extern float accelcal[3];
                flash2_fmc_write( accelcal[0] + 127 , accelcal[1] + 127);
                #endif
                
                #ifdef FLASH_SAVE1
								extern void flash_save( void);
                extern void flash_load( void);
								flash_save( );
                flash_load( );
                // reset flash numbers
                extern int number_of_increments[3][3];
                for( int i = 0 ; i < 3 ; i++)
                    for( int j = 0 ; j < 3 ; j++)
                        number_of_increments[i][j] = 0; 
                #endif
			    // reset loop time 
			    extern unsigned long lastlooptime;
			    lastlooptime = gettime();
						
		    }	
	
            if (command == GESTURE_DUD)
              {                  
								 #ifdef SWITCHABLE_FEATURE_3
                 extern int flash_feature_3;
                 flash_feature_3=!flash_feature_3;
                 ledblink = 2 - flash_feature_3;
                 pid_gestures_used = 1;								 
								 #endif
              }    
				
            if (command == GESTURE_UUU)
              {
                 #if defined (RX_DSMX_2048) || defined (RX_DSM2_1024) || defined (RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND)                  
                 extern int rx_bind_enable;
                 rx_bind_enable=!rx_bind_enable;
                 ledblink = 2 - rx_bind_enable;
                 pid_gestures_used = 1;  
                 #endif
								
							}    
			
            if (command == GESTURE_RRR)
              {								
								 #ifdef SWITCHABLE_FEATURE_1
                 extern int flash_feature_1;
                 flash_feature_1=!flash_feature_1;
                 ledblink = 2 - flash_feature_1;
                 pid_gestures_used = 1;								 
								 #endif
              }
 				
            if (command == GESTURE_LLL)
              {
                 #ifdef SWITCHABLE_FEATURE_2     
                 extern int flash_feature_2;
                 flash_feature_2=!flash_feature_2;
                 ledblink = 2 - flash_feature_2;
                 pid_gestures_used = 1;								 
								 #endif
              }
	             
            if (command == GESTURE_RRD)
              {
                  aux[CH_AUX1] = 1;
                  ledcommand = 1;
              }
            if (command == GESTURE_LLD)
              {
                  ledcommand = 1;
                  aux[CH_AUX1] = 0;
              }
            #ifdef PID_GESTURE_TUNING              
            if ( command >= GESTURE_UDR ) pid_gestures_used = 1;   
              
            if (command == GESTURE_UDU)
              {
                        // Cycle to next pid term (P I D)
                        ledblink = next_pid_term();
              }
            if (command == GESTURE_UDD)
              {
                        // Cycle to next axis (Roll Pitch Yaw)
                        ledblink = next_pid_axis();
              }
            if (command == GESTURE_UDR)
              {
                  // Increase by 10%
                        ledblink = increase_pid();
              }
            if (command == GESTURE_UDL)
              {
                        // Descrease by 10%
                  ledblink = decrease_pid();
              }
            // flash long on zero  
            if ( pid_gestures_used && ledblink == 0) ledcommand = 1; 
              
                // U D U - Next PID term
                // U D D - Next PID Axis
                // U D R - Increase value
                // U D L - Descrease value
               // ledblink = blink; //Will cause led logic to blink the number of times ledblink has stored in it.
                #endif

	  }
		#endif		
 }

