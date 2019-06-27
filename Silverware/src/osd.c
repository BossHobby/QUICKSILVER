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
LDFLAGS += -u _printf_float       <-add to build options - linker flag


// keep a static count variable for things that only need to be sent once
static uint8_t callSign_count = 0;
//grab out of scope variables for data that needs to be displayed
extern float vbattfilt;
extern float vbattfilt_corr;
extern float vbatt_comp;	
extern float lipo_cell_count;



// pilot name routine could look something like this...

callSign.active = 1;																	//struct elements should be populated from flash memory read at boot up in final design
strcpy(&callSign.text, "ALIENWHOOP");									//really sorting that out needs to come first.  Not yet sure where elements of struct should be initialized yet?
callSign.position_x = 7;			//random values for now
callSign.position_y = 7;
callSign.attribute = TEXT;

if (callSign.active == 1)		//check if call sign is a user selected elemet to display
{
	if (callSign_count < 1)		//check if it has already been sent once
	{
	osd_print(&callSign.text , callSign.attribute , callSign.position_x , callSign.position_y);
	callSign_count++;
	}
}

*/


} //end osd_display()

