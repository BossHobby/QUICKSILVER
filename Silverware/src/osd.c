#include "project.h"
#include "drv_max7456.h"
#include "string.h"


void osd_init(void)
{
	spi_max7456_init();
	max7456_init();
	osd_intro();
}

// maybe define reference coordinates for top and bottom, right left and middle?  Need to find these values.  
#define TL_X
#define TL_Y
#define TM_X
#define TM_Y
#define TR_X
#define TR_Y



// maybe a struct for elements to be displayed in OSD?
typedef struct screen_element_t
{
	uint8_t active;
	char text;
	uint8_t position_x;
	uint8_t position_y;
	uint8_t attribute;
}screen_element_s;

screen_element_s callSign;


/*screen elements characteristics written like registers in a 32bit binany number
except callsign which will take 6 addresses.  callsign bit 1 will be enable/disable,
bit 2 will be text/invert, and the remaining 30 bits will be 5 characters.  6 total addresses
will allow callsign text to fill the whole screen across
BIT
0			-		0 is display element active , 1 is display element inactive
1:2		-		00 is TEXT, 01 is INVERT, 11 is BLINK
3:7		-		the X screen position (column)
8:11	-		the Y screen position	(row)
12:15	-		not currently used
16:31	-		available for two binary ascii characters
*/
//Flash Variables - 32bit
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


void osd_display(void)
{
extern uint8_t lastsystem;
if (lastsystem > 1)  		//first check for video signal autodetect - run if necessary
{
	osd_checksystem();
}

//OSD ROUTINE HERE  - Still Brainstorming


/*				//commented out for now just in case


//NOTES:
//Printing a float may look something like this:
//osd_print( sprintf(buffer,"%.02f",(float) pidkp[0]) , TEXT,  10 , 10 );
//complier may need some poking
//LDFLAGS += -u _printf_float       <-add to build options - linker flag


// keep a static count variable for things that only need to be sent once
static uint8_t callSign_count = 0;
//grab out of scope variables for data that needs to be displayed
extern float vbattfilt;
extern float vbattfilt_corr;
extern float vbatt_comp;	
extern float lipo_cell_count;



// pilot name routine could look something like this...


if ((*callsign1 & 0x01) == 0x01)		//check if call sign is a user selected elemet to display
{
	if (callSign_count < 1)		//check if it has already been sent once
	{
	osd_print(&callSign.text , callSign.attribute , callSign.position_x , callSign.position_y);    //todo - gut the old struct stuff and use the new register method
	callSign_count++;
	}
}


*/

} //end osd_display()

