#include "drv_max7456.h"
#include "drv_time.h"
#include "project.h"
#include "stdio.h"
#include "string.h"
#include "profile.h"

#ifdef ENABLE_OSD

void osd_init(void) {
  spi_max7456_init(); //init spi
  max7456_init();     //init the max chip
  osd_intro();        //print the splash screen
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
uint8_t decode_attribute(uint32_t element) { //shifting right one bit and comparing the new bottom bit to the key above
  uint8_t decoded_element = ((element >> 1) & 0x01);
  if (decoded_element == 0x01)
    return INVERT;
  else
    return TEXT;
}

uint8_t decode_positionx(uint32_t element) { //shift 2 bits and grab the bottom 5
  return ((element >> 2) & 0x1F);            // this can be simplified to save memory if it debugs ok
}

uint32_t decode_positiony(uint32_t element) { //shift 7 bits and grab the bottom 4
  return ((element >> 7) & 0x0F);             // this can be simplified to save memory if it debugs ok
}

// case state variables for switch logic
uint8_t osd_display_phase = 2;
uint8_t osd_wizard_phase = 0;
uint8_t osd_menu_phase = 0;
uint8_t osd_display_element = 0;
uint8_t display_trigger = 0;
uint8_t last_lowbatt_state = 2;
uint8_t osd_cursor;

void osd_display_reset(void) {
  osd_wizard_phase = 0;    //reset the wizard
  osd_menu_phase = 0;      //reset menu to to main menu
  osd_display_phase = 2;   //jump to regular osd display next loop
  osd_display_element = 0; //start with first screen element
  last_lowbatt_state = 2;  //reset last lowbatt comparator
}

uint8_t osd_select;
uint8_t user_selection(uint8_t element, uint8_t total) {
  if (osd_cursor != element) {
    if (osd_cursor < 1)
      osd_cursor = 1;
    if (osd_cursor > total)
      osd_cursor = total;
  }
  if (osd_cursor == element && osd_select == 0) {
    return INVERT;
  } else {
    return TEXT;
  }
}

uint8_t adjust_selection(uint8_t element, uint8_t row) {
	if (osd_select == element && osd_cursor == row ){
		return INVERT;
	}else{
		return TEXT;
	}
}

extern int flash_feature_1; //currently used for auto entry into wizard menu
void osd_select_menu_item(void) {
  if (osd_select == 1 && flash_feature_1 == 1) //main mmenu
  {
    osd_select = 0; //reset the trigger

    switch (osd_cursor) {
    case 0:	//nothing happens here - case 0 is not used
      break;

    case 1:		//vtx
      break;

    case 2:		//pids
      break;

    case 3:		//filters
      break;

    case 4:		//rates
    	osd_cursor = 0;
    	osd_display_phase = 6;
    	osd_menu_phase = 0;
      break;

    case 5:		//flight modes
      break;

    case 6:
      break;

    case 7:
      break;

    case 8:
      flash_feature_1 = 0;
      break;

    case 9:
      osd_cursor = 0;
      osd_display_phase = 0;

#ifdef FLASH_SAVE1
      extern int pid_gestures_used;
      extern int ledcommand;
      extern void flash_save(void);
      extern void flash_load(void);
      pid_gestures_used = 0;
      ledcommand = 1;
      flash_save();
      flash_load();
      // reset flash numbers for pids
      extern int number_of_increments[3][3];
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
          number_of_increments[i][j] = 0;
      // reset loop time - maybe not necessary cause it gets reset in the next screen clear
      extern unsigned long lastlooptime;
      lastlooptime = gettime();
#endif
      break;
    }
  }
}

extern profile_t profile;
void osd_select_ratestype_item(void)
{//		like above function but to select between betaflight and silverware rate menus
  if (osd_select == 1) //stick was pushed right to select a rate type
  {
	osd_select = 0;	//reset the trigger
	switch(osd_cursor){
	case 1:
		osd_cursor = 0;	//reset the cursor
		profile.rate.mode = RATE_MODE_SILVERWARE;	//update profile
		osd_display_phase = 7;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	case 2:
		osd_cursor = 0;
		profile.rate.mode = RATE_MODE_BETAFLIGHT;
		osd_display_phase = 8;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	}
  }
}

void osd_adjust_silverwarerates_item(void)
{
	if(osd_select > 3) {
		osd_select = 3;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	switch(osd_cursor){
	case 1:	//adjust row 1 items based on osd_select value and up/down osd gestures
		break;
	case 2:	//adjust row 2 items based on osd_select value and up/down osd gestures
		break;
	case 3:	//adjust row 3 items based on osd_select value and up/down osd gestures
		break;
	case 4: //save&exit silverware rates
		if (osd_select == 1){
	      osd_cursor = 0;
	      osd_display_phase = 0;
		  #ifdef FLASH_SAVE1
	      extern int pid_gestures_used;
	      extern int ledcommand;
	      extern void flash_save(void);
	      extern void flash_load(void);
	      pid_gestures_used = 0;
	      ledcommand = 1;
	      flash_save();
	      flash_load();
	      // reset flash numbers for pids
	      extern int number_of_increments[3][3];
	      for (int i = 0; i < 3; i++)
	        for (int j = 0; j < 3; j++)
	          number_of_increments[i][j] = 0;
	      // reset loop time - maybe not necessary cause it gets reset in the next screen clear
	      extern unsigned long lastlooptime;
	      lastlooptime = gettime();
		  #endif
		}
		break;
	}
}

void osd_display(void) {
  //first check if video signal autodetect needs to run - run if necessary
  extern uint8_t lastsystem; //initialized at 99 for none then becomes 0 or 1 for ntsc/pal
  if (lastsystem > 1)        //if no camera was detected at boot up
  {
    osd_checksystem(); //try to detect camera
    if (lastsystem < 2) {
      osd_display_reset(); //camera has been detected while in the main loop and screen has been cleared again - reset screen cases
    }
  }

  //************OSD MENU DISPLAY ROUTINES HERE*************

  //grab out of scope variables for data that needs to be displayed
  //extern int flash_feature_1;
  //extern float vbattfilt;
  //extern float vbattfilt_corr;
  extern float vbatt_comp;
  extern float lipo_cell_count;
  extern int lowbatt;
  //just some values to test position/attribute/active until we start saving them to flash with the osd manu
  *callsign1 = 0xAB;        //	0001 01010 11
  *fuelgauge_volts = 0x72D; //‭  1110 01011 01‬

  switch (osd_display_phase) //phase starts at 2, RRR gesture subtracts 1 to enter the menu, RRR again or DDD subtracts 1 to clear the screen and return to regular display
  {
  case 0: //osd screen clears, resets to regular display, and resets wizard and menu starting points
    osd_clear();
    osd_display_reset();
    extern unsigned long lastlooptime;
    lastlooptime = gettime();
    break; //screen has been cleared for this loop - break out of display function

  case 1:                //osd menu is active
    if (flash_feature_1) //setup wizard
    {
      switch (osd_menu_phase) {
      case 0:
        osd_clear();
        extern unsigned long lastlooptime;
        lastlooptime = gettime();
        osd_menu_phase++;
        break;
      case 1:
        osd_print("MENU", INVERT, 13, 1); //function call returns text or invert, gets passed # of elements for wrap around,
        osd_menu_phase++;                 //and which element number this is
        break;
      case 2:
        osd_print("VTX", user_selection(1, 9), 7, 3); //selection_status (1,9)
        osd_menu_phase++;
        break;
      case 3:
        osd_print("PIDS", user_selection(2, 9), 7, 4);
        osd_menu_phase++;
        break;
      case 4:
        osd_print("FILTERS", user_selection(3, 9), 7, 5);
        osd_menu_phase++;
        break;
      case 5:
        osd_print("RATES", user_selection(4, 9), 7, 6);
        osd_menu_phase++;
        break;
      case 6:
        osd_print("FLIGHT MODES", user_selection(5, 9), 7, 7);
        osd_menu_phase++;
        break;
      case 7:
        osd_print("OSD ELEMENTS", user_selection(6, 9), 7, 8);
        osd_menu_phase++;
        break;
      case 8:
        osd_print("SPECIAL FEATURES", user_selection(7, 9), 7, 9);
        osd_menu_phase++;
        break;
      case 9:
        osd_print("SETUP WIZARD", user_selection(8, 9), 7, 10);
        osd_menu_phase++;
        break;
      case 10:
        osd_print("SAVE + EXIT", user_selection(9, 9), 7, 11);
        osd_menu_phase++;
        break;
      case 11:
        osd_select_menu_item();
        break;
      }
    } else {
      switch (osd_wizard_phase) {
      case 0:
        osd_clear();
        extern unsigned long lastlooptime;
        lastlooptime = gettime();
        osd_wizard_phase++;
        break;
      case 1:
        osd_print("SETUP WIZARD", INVERT, 9, 1);
        osd_wizard_phase++;
        break;
      case 2:
        osd_print("PROPS OFF", BLINK, 7, 6);
        osd_wizard_phase++;
        break;
      case 3:
        osd_print("THROTTLE UP", TEXT, 7, 8);
        osd_wizard_phase++;
        break;
      case 4:
        osd_print("TO CONTINUE", TEXT, 7, 9);
        osd_wizard_phase++;
        break;
      case 5:
        break;
      }
    }
    break; //osd menu or wizard has been displayed for this loop	- break out of display function

  case 2: //regular osd display
    switch (osd_display_element) {
    case 0:
      if ((*callsign1 & 0x01) == 0x01)
        osd_print("ALIENWHOOP", decode_attribute(*callsign1), decode_positionx(*callsign1), decode_positiony(*callsign1)); //todo - needs to be pulled from the new register flash method
      osd_display_element++;
      break; //screen has been displayed for this loop - break out of display function

    case 1:
      if ((*fuelgauge_volts & 0x01) == 1) {
        uint8_t osd_fuelgauge_volts[5];
        fast_fprint(osd_fuelgauge_volts, 4, vbatt_comp, 1);
        osd_fuelgauge_volts[4] = 'V';
        osd_print_data(osd_fuelgauge_volts, 5, decode_attribute(*fuelgauge_volts), decode_positionx(*fuelgauge_volts) + 3, decode_positiony(*fuelgauge_volts));
      }
      osd_display_element++;
      break;

    case 2:
      if ((*fuelgauge_volts & 0x01) == 1) {
        if (lowbatt != last_lowbatt_state) {
          uint8_t osd_cellcount[2] = {lipo_cell_count + 48, 'S'};
          if (!lowbatt) {
            osd_print_data(osd_cellcount, 2, decode_attribute(*fuelgauge_volts), decode_positionx(*fuelgauge_volts), decode_positiony(*fuelgauge_volts));
          } else {
            osd_print_data(osd_cellcount, 2, BLINK | INVERT, decode_positionx(*fuelgauge_volts), decode_positiony(*fuelgauge_volts));
          }
          last_lowbatt_state = lowbatt;
        }
      }
      osd_display_element++;
      break;

    case 3:
      display_trigger++;
      if (display_trigger == 0)
        osd_display_element = 1;
      break;
    }
    break;

  case 3:		//vtx menu
    break;

  case 4:		//pids menu
	break;

  case 5:		//filters menu
    break;

  case 6:		//main rates menu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("RATES", INVERT, 13, 1); //function call returns text or invert, gets passed # of elements for wrap around,
    	  osd_menu_phase++;                 //and which element number this is
    	  break;
      case 2:
          osd_print("SILVERWARE", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("BETAFLIGHT", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  osd_select_ratestype_item();
    	  break;
      }
    break;

  case 7:		//silverware rates submenu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("SILVERWARE RATES", INVERT, 7, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("ROLL", TEXT, 14, 4);
          osd_menu_phase++;
          break;
      case 3:
    	  osd_print("PITCH", TEXT, 19, 4);
          osd_menu_phase++;
          break;
      case 4:
    	  osd_print("YAW", TEXT, 25, 4);
          osd_menu_phase++;
          break;
      case 5:
          osd_print("RATE", user_selection(1, 4), 2, 6);
          osd_menu_phase++;
          break;
      case 6:
    	  osd_print("ACRO EXPO", user_selection(2, 4), 2, 7);
          osd_menu_phase++;
          break;
      case 7:
          osd_print("ANGLE EXPO", user_selection(3, 4), 2, 8);
          osd_menu_phase++;
          break;
      case 8:
          osd_print("SAVE AND EXIT", user_selection(4, 4), 2, 14);
          osd_menu_phase++;
          break;
      case 9:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_roll_rate[4];
          fast_fprint(osd_roll_rate, 5, profile.rate.silverware.max_rate.roll, 0);
          osd_print_data(osd_roll_rate, 4, adjust_selection(1, 1), 15, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_pitch_rate[4];
          fast_fprint(osd_pitch_rate, 5, profile.rate.silverware.max_rate.pitch, 0);
          osd_print_data(osd_pitch_rate, 4, adjust_selection(2, 1), 20, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_yaw_rate[4];
          fast_fprint(osd_yaw_rate, 5, profile.rate.silverware.max_rate.yaw, 0);
          osd_print_data(osd_yaw_rate, 4, adjust_selection(3, 1), 25, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_roll[4];
          fast_fprint(osd_acro_expo_roll, 4, profile.rate.silverware.acro_expo.roll, 2);
          osd_print_data(osd_acro_expo_roll, 4, adjust_selection(1, 2), 15, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_pitch[4];
          fast_fprint(osd_acro_expo_pitch, 4, profile.rate.silverware.acro_expo.pitch, 2);
          osd_print_data(osd_acro_expo_pitch, 4, adjust_selection(2, 2), 20, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_yaw[4];
          fast_fprint(osd_acro_expo_yaw, 4, profile.rate.silverware.acro_expo.yaw, 2);
          osd_print_data(osd_acro_expo_yaw, 4, adjust_selection(3, 2), 25, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_roll[4];
          fast_fprint(osd_angle_expo_roll, 4, profile.rate.silverware.angle_expo.roll, 2);
          osd_print_data(osd_angle_expo_roll, 4, adjust_selection(1, 3), 15, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_pitch[4];
          fast_fprint(osd_angle_expo_pitch, 4, profile.rate.silverware.angle_expo.pitch, 2);
          osd_print_data(osd_angle_expo_pitch, 4, adjust_selection(2, 3), 20, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_yaw[4];
          fast_fprint(osd_angle_expo_yaw, 4, profile.rate.silverware.angle_expo.yaw, 2);
          osd_print_data(osd_angle_expo_yaw, 4, adjust_selection(3, 3), 25, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_adjust_silverwarerates_item();
    	  break;
      }
    break;

  case 8:		//betaflight rates submenu
    break;

  case 9:		//flight modes menu
    break;

  case 10:		//osd elements menu
    break;

  case 11:		//special features menu
  	break;

  }

} //end osd_display()

#endif
