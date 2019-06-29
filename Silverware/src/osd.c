#include "project.h"
#include "drv_max7456.h"
#include "string.h"
#include "stdio.h"


void osd_init(void)
{
	spi_max7456_init();
	max7456_init();
	osd_intro();
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
unsigned long *callsign1 = osd_element;

unsigned long *fuelgauge_volts = (osd_element + 6);
unsigned long *filtered_volts = (osd_element + 7);
unsigned long *exact_volts = (osd_element + 8);


//  screen element register decoding functions
uint8_t decode_attribute (uint32_t element)
{	//shifting right one bit and comparing the new bottom bit to the key above
	uint8_t decoded_element = ((element>>1) & 0x01);
	if( decoded_element == 0x01)return INVERT;
	else return TEXT;
}

uint8_t decode_positionx (uint32_t element)
{	//shift 2 bits and grab the bottom 5
	return ((element>>2) & 0x1F); // this can be simplified to save memory if it debugs ok
//	return decoded_element;
}

uint32_t decode_positiony (uint32_t element)
{	//shift 7 bits and grab the bottom 4
	uint32_t decoded_element = ((element>>7) & 0x0F); // this can be simplified to save memory if it debugs ok
	return decoded_element;
}






void osd_display(void)
{

//first check for video signal autodetect - run if necessary	
extern uint8_t lastsystem;
if (lastsystem > 1)  		
{
	osd_checksystem();
}

//OSD ROUTINES HERE

//static variables to keep the state of menus
static uint8_t osd_menu_active = 0;    // off -0 , on - 1

// keep a static count variable for things that only need to be sent once
static uint8_t callSign_count = 0;

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
//
// pilot callsign
*callsign1 = 0x2B;   //just a value to test position/attribute/active
if ((*callsign1 & 0x01) == 0x01)		//check if call sign is a user selected elemet to display
{
	if (callSign_count < 1)		//check if it has already been sent once
	{
	osd_print("ALIENWHOOP" , decode_attribute(*callsign1) , decode_positionx(*callsign1) , decode_positiony(*callsign1));    //todo - gut the old struct stuff and use the new register method
	callSign_count++;
							//	extern void flash_save( void);					//simulates save command for debugging
              //  extern void flash_load( void);
							//	flash_save( );
              //  flash_load( );
	}
}


//fuelgauge volts
*fuelgauge_volts = 0x731;		//another test value from the simulated register	
static float last_fuelgauge_volts;
if (vbatt_comp != last_fuelgauge_volts)
{
last_fuelgauge_volts = vbatt_comp;
char osd_fuelgauge_volts[4];
sprintf(osd_fuelgauge_volts,"%.1fV",(float) vbatt_comp);
osd_print( osd_fuelgauge_volts , decode_attribute(*fuelgauge_volts),  decode_positionx(*fuelgauge_volts) +3 , decode_positiony(*fuelgauge_volts) );
char osd_cellcount[2];
sprintf(osd_cellcount,"%.fS",(float) lipo_cell_count);
if (!lowbatt){
osd_print( osd_cellcount ,decode_attribute(*fuelgauge_volts) ,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );
}else{
osd_print( osd_cellcount , BLINK | INVERT,  decode_positionx(*fuelgauge_volts) , decode_positiony(*fuelgauge_volts) );	
}
}
//NOTES:
//complier may need some poking for sprintf
//LDFLAGS += -u _printf_float       <-add to build options - linker flag

//filtered volts

//exact volts

//armed time

//other stuff


} //end osd_display()

