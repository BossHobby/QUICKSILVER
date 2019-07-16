#include "project.h"
#include "drv_max7456.h"
#include "string.h"
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






void osd_display(void)
{

//static variables to keep the state of menus
static uint8_t osd_menu_active = 0;    // off -0 , on - 1

// keep a static count variable for things that only need to be sent once
static uint8_t callSign_count = 0;
	
//first check for video signal autodetect - run if necessary	
extern uint8_t lastsystem;		//initialized at 99 for none then becomes 0 or 1 for ntsc/pal
	if (lastsystem > 1)  		
	{
	osd_checksystem();
		if (lastsystem < 2)  //camera has been detected while in the main loop and screen has been cleared again - reset static variables for send once screen elements
		{
		callSign_count = 0;	
		}
	}

//************OSD ROUTINES HERE*************


//grab out of scope variables for data that needs to be displayed
extern float vbattfilt;
extern float vbattfilt_corr;
extern float vbatt_comp;	
extern float lipo_cell_count;
extern int lowbatt;



//  *********************OSD MENU
if (osd_menu_active)
{
	//display osd menu, listen for stick movements, and change the values of the pointers based on user selection
}



// *********************OSD REGULAR DISPLAY

// pilot callsign

*callsign1 = 0xAB;   //	1 01010 11			just a value to test position/attribute/active
if ((*callsign1 & 0x01) == 0x01)		//check if call sign is a user selected elemet to display
{
	if (callSign_count < 1)		//check if it has already been sent once
	{
	osd_print("ALIENWHOOP" , decode_attribute(*callsign1) , decode_positionx(*callsign1) , decode_positiony(*callsign1));    //todo - needs to be pulled from the new register flash method	
		callSign_count++;
	}
}


//fuelgauge volts

*fuelgauge_volts = 0x72D;		//	1110 01011 01			another test value from the simulated register	
static uint8_t vbat_trigger = 0;
vbat_trigger++;
if (vbat_trigger == 255)
{
uint8_t osd_fuelgauge_volts[5];
fast_fprint(osd_fuelgauge_volts, 4, vbatt_comp, 1);
osd_fuelgauge_volts[4]='V';
osd_print_data( osd_fuelgauge_volts , 5 , decode_attribute(*fuelgauge_volts),  decode_positionx(*fuelgauge_volts) +3 , decode_positiony(*fuelgauge_volts) );
}

//cellcount is part of voltage element	but printed on a different trigger
static uint8_t lowbat_trigger = 10;
lowbat_trigger++;
if (lowbat_trigger == 255)
{
uint8_t osd_cellcount[2];
osd_cellcount[0] = lipo_cell_count + 48;
osd_cellcount[1] = 'S';
if (!lowbatt){
osd_print_data( osd_cellcount , 2 , decode_attribute(*fuelgauge_volts) ,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );
}else{
osd_print_data( osd_cellcount , 2 , BLINK | INVERT,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );	
}
}


//filtered volts

//exact volts

//armed time

//other stuff


} //end osd_display()

#endif
