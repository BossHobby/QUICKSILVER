#include "project.h"
#include "drv_max7456.h"
#include "string.h"
#include "drv_time.h"
#include "stdio.h"

#ifdef ENABLE_OSD

void osd_init(void)
{
	spi_max7456_init();		//init spi
	max7456_init();				//init the max chip
	osd_intro();					//print the splash screen
}


/*screen elements characteristics written like registers in a 32bit binany number
except callsign which will take 6 addresses.  callsign bit 1 will be enable/disable,
bit 2 will be text/invert, and the remaining 30 bits will be 5 characters.  6 total addresses
will allow callsign text to fill the whole screen across
BIT
0			-		0 is display element inactive , 1 is display element active
1			-		0 is TEXT, 1 is INVERT
2:6		-		the X screen position (column)
7:10	-		the Y screen position	(row)
11:15	-		not currently used
16:31	-		available for two binary ascii characters
*/

//Flash Variables - 32bit					# of osd elements and flash memory start position in defines.h
unsigned long osd_element[OSD_NUMBER_ELEMENTS];
/*
elements 0-5 - Call Sign
element 6 Fuel Gauge volts
element 7 Filtered Volts
element 8 Exact Volts
*/

//pointers to flash variable array 
unsigned long *callsign1 = osd_element;
unsigned long *callsign2 = (osd_element + 1);
unsigned long *callsign3 = (osd_element + 2);
unsigned long *callsign4 = (osd_element + 3);
unsigned long *callsign5 = (osd_element + 4);
unsigned long *callsign6 = (osd_element + 5);
unsigned long *fuelgauge_volts = (osd_element + 6);
unsigned long *filtered_volts = (osd_element + 7);
unsigned long *exact_volts = (osd_element + 8);


//screen element register decoding functions
uint8_t decode_attribute (uint32_t element)
{	//shifting right one bit and comparing the new bottom bit to the key above
	uint8_t decoded_element = ((element>>1) & 0x01);
	if( decoded_element == 0x01)return INVERT;
	else return TEXT;
}

uint8_t decode_positionx (uint32_t element)
{	//shift 2 bits and grab the bottom 5
	return ((element>>2) & 0x1F); // this can be simplified to save memory if it debugs ok

}

uint32_t decode_positiony (uint32_t element)
{	//shift 7 bits and grab the bottom 4
	return ((element>>7) & 0x0F); // this can be simplified to save memory if it debugs ok
}



// case state variables for switch logic
uint8_t osd_display_phase = 2;
uint8_t osd_wizard_phase = 0;
uint8_t osd_menu_phase = 0;
uint8_t osd_display_element = 0;
uint8_t display_trigger = 0;
uint8_t last_lowbatt_state = 2;

void osd_display_reset(void)
{
osd_wizard_phase = 0;			//reset the wizard
osd_menu_phase = 0;				//reset menu to to main menu
osd_display_phase = 2;		//jump to regular osd display next loop	
osd_display_element = 0;	//start with first screen element
last_lowbatt_state = 2;		//reset last lowbatt comparator
}


void osd_display(void)
{
//first check for video signal autodetect - run if necessary	
extern uint8_t lastsystem;		//initialized at 99 for none then becomes 0 or 1 for ntsc/pal
if (lastsystem > 1)  				//if no camera was detected at boot up
{
	osd_checksystem();					//try to detect camera						
	if (lastsystem < 2) osd_display_reset(); 			//camera has been detected while in the main loop and screen has been cleared again - reset screen cases
}

//************OSD ROUTINES HERE*************


//grab out of scope variables for data that needs to be displayed
extern int flash_feature_1;  //currently used for auto entry into wizard menu
//extern float vbattfilt;
//extern float vbattfilt_corr;
extern float vbatt_comp;	
extern float lipo_cell_count;
extern int lowbatt;
//just some values to test position/attribute/active until we start saving them to flash with the osd manu
*callsign1 = 0xAB;   //	1 01010 11			
*fuelgauge_volts = 0x72D;


switch(osd_display_phase)		//phase starts at 2, RRR gesture subtracts 1 to enter the menu, RRR again or DDD subtracts 1 to clear the screen and return to regular display
{
case 0:											//osd screen clears, resets to regular display, and resets wizard and menu starting points
	osd_clear();
	osd_display_reset();
	extern unsigned long lastlooptime;
	lastlooptime = gettime();	
	break;										//screen has been cleared for this loop - break out of display function

case 1:											//osd menu is active
	if (flash_feature_1)			//setup wizard
	{
	switch( osd_menu_phase ) {
		case 0:
			osd_clear();
			extern unsigned long lastlooptime;
			lastlooptime = gettime();				
			osd_menu_phase++;
			break;
		case 1:
			osd_print("MENU" , INVERT , 13 , 1);
			osd_menu_phase++;
			break;
		case 2:
			osd_print("VTX" , TEXT , 7, 3);
			osd_menu_phase++;
			break;	
		case 3:
			osd_print("PIDS" , TEXT , 7, 4);
			osd_menu_phase++;
			break;
		case 4:
			osd_print("FILTERS" , TEXT , 7, 5);
			osd_menu_phase++;
			break;
		case 5:
			osd_print("RATES" , TEXT , 7, 6);
			osd_menu_phase++;
			break;
		case 6:
			osd_print("FLIGHT MODES" , TEXT , 7, 7);
			osd_menu_phase++;
			break;
		case 7:
			osd_print("OSD ELEMENTS" , TEXT , 7, 8);
			osd_menu_phase++;
			break;
		case 8:
			osd_print("SPECIAL FEATURES" , TEXT , 7, 9);
			osd_menu_phase++;
			break;	
		case 9:
			osd_print("SETUP WIZARD" , TEXT , 7, 10);
			osd_menu_phase++;
			break;
		case 10:
			osd_print("SAVE + EXIT" , TEXT , 7, 11);
			osd_menu_phase++;
			break;
		case 11:
			break;
	}		
	}else{
	switch( osd_wizard_phase ) {
		case 0:
			osd_clear();
			extern unsigned long lastlooptime;
			lastlooptime = gettime();				
			osd_wizard_phase++;
			break;
		case 1:
			osd_print("SETUP WIZARD" , INVERT , 9, 1);
			osd_wizard_phase++;
			break;
		case 2:
			osd_print("PROPS OFF" , BLINK , 7, 6);
			osd_wizard_phase++;
			break;	
		case 3:
			osd_print("THROTTLE UP" , TEXT , 7, 8);
			osd_wizard_phase++;
			break;
		case 4:
			osd_print("TO CONTINUE" , TEXT , 7, 9);
			osd_wizard_phase++;
			break;
		case 5:
			break;
	}
	}
	break;											//osd menu or wizard has been displayed for this loop	- break out of display function


case 2:												//regular osd display
	switch(osd_display_element)
	{
		case 0:
			if ((*callsign1 & 0x01) == 0x01) osd_print("ALIENWHOOP" , decode_attribute(*callsign1) , decode_positionx(*callsign1) , decode_positiony(*callsign1));    //todo - needs to be pulled from the new register flash method	
			osd_display_element++;
			break;											//screen has been displayed for this loop - break out of display function
		
		case 1:
			if((*fuelgauge_volts & 0x01) == 1)
			{
			uint8_t osd_fuelgauge_volts[5];
			fast_fprint(osd_fuelgauge_volts, 4, vbatt_comp, 1);
			osd_fuelgauge_volts[4]='V';
			osd_print_data( osd_fuelgauge_volts , 5 , decode_attribute(*fuelgauge_volts),  decode_positionx(*fuelgauge_volts) +3 , decode_positiony(*fuelgauge_volts) );
			}
			osd_display_element++;
			break;
			
		case 2:
			if((*fuelgauge_volts & 0x01) == 1)
			{ 
			if (lowbatt != last_lowbatt_state)
			{
			uint8_t osd_cellcount[2] = {lipo_cell_count + 48 , 'S'};
			if (!lowbatt){
			osd_print_data( osd_cellcount , 2 , decode_attribute(*fuelgauge_volts) ,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );
			}else{
			osd_print_data( osd_cellcount , 2 , BLINK | INVERT,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );	
			}
			last_lowbatt_state = lowbatt;
			}
			}
			osd_display_element++;
			break;

		case 3:
			display_trigger++;
			if (display_trigger == 0) osd_display_element = 1;
			break;	
	}	
	break;
}

} //end osd_display()

#endif
