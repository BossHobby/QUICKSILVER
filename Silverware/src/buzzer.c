
#include "project.h"
#include "config.h"
#include "drv_time.h"
#include "buzzer.h"
#include "defines.h"

#ifdef BUZZER_ENABLE

#define PIN_OFF( port , pin ) GPIO_ResetBits( port , pin)
#define PIN_ON( port , pin ) GPIO_SetBits( port , pin)


int gpio_init_buzzer(void)
{

		// set gpio pin as output
		GPIO_InitTypeDef GPIO_InitStructure;

		// common settings to set ports
		GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

		GPIO_Init(BUZZER_PIN_PORT,&GPIO_InitStructure);
		return 1;
}


void buzzer()
{
	extern int failsafe;
	extern int lowbatt;
	extern int rxmode;
	extern unsigned long lastlooptime;
	
	static int toggle;
	static unsigned long buzzertime;
	static int buzzer_init = 0;
	
	unsigned long pulse_rate;
	
	// waits 5 seconds
	// before configuring the gpio buzzer pin to ensure
	// there is time to program the chip (if using SWDAT or SWCLK)

	extern char aux[];
	if ( lowbatt || failsafe || aux[BUZZER_ENABLE] )
	{
		unsigned long time = gettime();
		if ( buzzertime == 0)
			buzzertime = time;
		else
		{
			
			// rank lowbatt > failsafe > throttle
			if (lowbatt)
				pulse_rate = 200000; // 1/5th second
			else if (failsafe)
				pulse_rate = 400000; // 2/5ths second
			else
				pulse_rate = 600000; // 3/5ths second

			// start the buzzer if timeout has elapsed
			if ( time - buzzertime > BUZZER_DELAY || lowbatt)
			{
				// initialize pin only after minimum 10 seconds from powerup
				if ( !buzzer_init && time >  10e6)
				{
					buzzer_init = gpio_init_buzzer();
				}
				
				// don't continue if buzzer not initialized
				if ( !buzzer_init ) return;
				
				
				// enable buzzer
				if (time%pulse_rate>pulse_rate/2)
				{
					if ( toggle  ) // cycle the buzzer
					{
					PIN_ON( BUZZER_PIN_PORT, BUZZER_PIN); // on
					}
					else 
					{
					PIN_OFF( BUZZER_PIN_PORT, BUZZER_PIN); // off
					}
					toggle = !toggle;
				}
				else
				{
					PIN_OFF(BUZZER_PIN_PORT, BUZZER_PIN );
				}
			}
			
		}

	}
	else
	{
		buzzertime = 0;
		// set buzzer to off if beeping condition stopped
		if (buzzer_init) PIN_OFF(BUZZER_PIN_PORT, BUZZER_PIN );
		
	}
	
}

#endif




