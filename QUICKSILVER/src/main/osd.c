#include "drv_max7456.h"
#include "drv_time.h"
#include "project.h"
#include "stdio.h"
#include "string.h"
#include "profile.h"
#include "float.h"
#include "util.h"
#include "rx.h"
#include "osd.h"

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
//******************************************************************************************************************************


//******************************************************************************************************************************
// case & state variables for switch logic and profile adjustments

extern int flash_feature_1; //currently used for auto entry into wizard menu
extern profile_t profile;
uint8_t osd_display_phase = 2;
uint8_t osd_wizard_phase = 0;
uint8_t osd_menu_phase = 0;
uint8_t osd_display_element = 0;
uint8_t display_trigger = 0;
uint8_t last_lowbatt_state = 2;
uint8_t osd_cursor;
uint8_t osd_select;
uint8_t increase_osd_value;
uint8_t decrease_osd_value;

#define BF_PIDS 0
const float bf_pids_increments[] = {0.0015924, 0.0015924, 0.0031847, 0.02, 0.02, 0.02, 0.0083333, 0.0083333, 0.0083333};
#define SW_RATES 1
const float sw_rates_increments[] = {10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
#define ROUNDED 2
const float rounded_increments[] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

#define MAIN_MENU 1
#define SUB_MENU 0

//main menu maps
const char main_menu_labels[10][21] = {"MENU","VTX","PIDS","FILTERS","RATES","FLIGHT MODES","OSD ELEMENTS","SPECIAL FEATURES","SETUP WIZARD","SAVE AND EXIT"};
const uint8_t main_menu_positions[10][2] = { {13, 1},{7, 3},{7, 4},{7, 5},{7, 6},{7, 7},{7, 8},{7, 9},{7, 10},{7, 11} };
const uint8_t main_menu_map[] = {11, 3, 5, 6, 9, 10, 12};		//case numbers for {vtx, pids, filters, rates, flight modes, osd elements, special features}

//pid profiles submenu map
const char pid_profiles_labels[3][21] = { {"PID PROFILES"},{"PID PROFILE 1"},{"PID PROFILE 2"} };
const uint8_t pid_profiles_positions[3][2] = { {9, 1},{7, 4},{7, 5} };
const uint8_t pid_submenu_map[] = {4, 4};	//describes the menu case to call next for each submenu option

// pids profiles map
const char pid_profile1_labels[8][21] = { {"PID PROFILE 1"},{"ROLL"},{"PITCH"},{"YAW"},{"KP"},{"KI"},{"KD"},{"SAVE AND EXIT"} };
const char pid_profile2_labels[8][21] = { {"PID PROFILE 2"},{"ROLL"},{"PITCH"},{"YAW"},{"KP"},{"KI"},{"KD"},{"SAVE AND EXIT"} };
const uint8_t pid_profile_positions[8][2] = { {9, 1},{10, 4},{16, 4},{23, 4},{4, 6},{4, 7},{4, 8},{2, 14} };
const uint8_t pid_profile_grid[9][2] = { {1, 1},{2, 1},{3, 1},{1, 2},{2, 2},{3, 2},{1, 3},{2, 3},{3, 3} };
const uint8_t pid_profile_data_positions[9][2] = { {10, 6},{16, 6},{22, 6},{10, 7},{16, 7},{22, 7},{10, 8},{16, 8},{22, 8} };
const uint8_t pid_profile_data_index[9][2] = { {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2} };
const uint8_t pid_scale_index[9] = {0, 0, 1, 2, 2, 2, 3, 3, 3};

//rates submenu map
const uint8_t rates_submenu_map[] = {7, 8};

//flight modes map
const char flight_modes_labels[12][21] = {"FLIGHT MODES","ARMING","IDLE UP","LEVELMODE","RACEMODE","HORIZON","STICK BOOST","HIGH RATES","BUZZER","TURTLE","MOTOR TEST","SAVE AND EXIT"};
const uint8_t flight_modes_positions[12][2] = { {9, 1},{4, 2},{4, 3},{4, 4},{4, 5},{4, 6},{4, 7},{4, 8},{4, 9},{4, 10},{4, 11},{4, 14} };
const uint8_t flight_modes_data_positions[10][2] = { {17, 2}, {17, 3}, {17, 4}, {17, 5}, {17, 6}, {17, 7}, {17, 8}, {17, 9}, {17, 10}, {17, 11} };
const uint8_t flight_modes_aux_limits[] = {11, 14, 14, 14, 14, 14, 14, 14, 14, 14};	//from aux_channel_t
const uint8_t flight_modes_aux_items[] = {0, 1, 2, 3, 4, 5, 7, 9, 10, 12};			//from aux_function_t
const uint8_t flight_modes_grid[10][2] = { {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10} };

//special features menu map
const uint8_t special_features_map[] = {13};					//case numbers for {stickboost} ...adding more soon

//stick boost submenu map
const uint8_t stickboost_submenu_map[] = {14, 14};
//******************************************************************************************************************************


//******************************************************************************************************************************
// osd utility functions

void osd_display_reset(void) {
  osd_wizard_phase = 0;    //reset the wizard
  osd_menu_phase = 0;      //reset menu to to main menu
  osd_display_phase = 2;   //jump to regular osd display next loop
  osd_display_element = 0; //start with first screen element
  last_lowbatt_state = 2;  //reset last lowbatt comparator
}

void osd_save_exit(void){
	osd_select = 0;
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

void osd_clear_screen(void){
	osd_clear();
	extern unsigned long lastlooptime;
	lastlooptime = gettime();
}

// this may be able to go away soon
uint8_t user_selection(uint8_t element, uint8_t total) {
 // if (osd_cursor != element) {
    if (osd_cursor < 1)
      osd_cursor = 1;
    if (osd_cursor > total)
      osd_cursor = total;
  //}
  if ((osd_cursor == element && osd_select == 0) || element == 0) {
    return INVERT;
  } else {
    return TEXT;
  }
}

uint8_t user_select(uint8_t active_elements, uint8_t total_elements){
    if (osd_cursor < 1) osd_cursor = 1;
    if (osd_cursor > active_elements) osd_cursor = active_elements;
    uint8_t inactive_elements = total_elements - active_elements;
    if (osd_menu_phase == 1) return INVERT;
    if (osd_menu_phase > 1 && osd_menu_phase <= inactive_elements) return TEXT;
    if (osd_cursor == (osd_menu_phase - inactive_elements)  && osd_select == 0){
    	return INVERT;
    }else{
    	return TEXT;
    }
}

uint8_t grid_selection(uint8_t element, uint8_t row) {
	if (osd_select == element && osd_cursor == row ){
		return INVERT;
	}else{
		return TEXT;
	}
}

uint16_t pid_scale(float pid_rate, uint8_t pid_scale_index){
	if (pid_scale_index == 0){	//	kP roll/pitch	increment by 0.0015923566878981f
		uint16_t scaled_pid_value = round_num(pid_rate * 628.0f);
		return scaled_pid_value;
	}
	if (pid_scale_index == 1){	//	kP yaw			increment by 0.0031847133757962f
		uint16_t scaled_pid_value = round_num(pid_rate * 314.0f);
		return scaled_pid_value;
	}
	if (pid_scale_index == 2){	//	kI				increment by 0.02f
		uint16_t scaled_pid_value = round_num(pid_rate * 50.0f);
		return scaled_pid_value;
	}
	if (pid_scale_index == 3){	//	kD				increment by 0.0083333333333333f
		uint16_t scaled_pid_value = round_num(pid_rate * 120.0f);
		return scaled_pid_value;
	}
	return 0;
}

float adjust_rounded_float(float input, float adjust_amount){
	float value = (int)(input * 100.0f + 0.5f);
	if (increase_osd_value){
		increase_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		return (float)(value+(100.0f * adjust_amount)) / 100.0f;
	}
	if (decrease_osd_value){
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		return (float)(value-(100.0f * adjust_amount)) / 100.0f;
	}
	return input;
}


const char* get_aux_status (int input){
	static char* respond[] = {"CHANNEL 5  ", "CHANNEL 6  ", "CHANNEL 7  ", "CHANNEL 8  ", "CHANNEL 9  ", "CHANNEL 10 ", "CHANNEL 11 ", "CHANNEL 12 ", "CHANNEL 13 ", "CHANNEL 14 ", "CHANNEL 15 ", "CHANNEL 16 ", "GESTURE AUX", "ALWAYS ON  ", "ALWAYS OFF ", "ERROR      "};
	if(input == AUX_CHANNEL_0) return respond[0];
	if(input == AUX_CHANNEL_1) return respond[1];
	if(input == AUX_CHANNEL_2) return respond[2];
	if(input == AUX_CHANNEL_3) return respond[3];
	if(input == AUX_CHANNEL_4) return respond[4];
	if(input == AUX_CHANNEL_5) return respond[5];
	if(input == AUX_CHANNEL_6) return respond[6];
	if(input == AUX_CHANNEL_7) return respond[7];
	if(input == AUX_CHANNEL_8) return respond[8];
	if(input == AUX_CHANNEL_9) return respond[9];
	if(input == AUX_CHANNEL_10) return respond[10];
	if(input == AUX_CHANNEL_11) return respond[11];
	if(input == AUX_CHANNEL_12) return respond[12];
	if(input == AUX_CHANNEL_ON) return respond[13];
	if(input == AUX_CHANNEL_OFF) return respond[14];
	return respond[15];
}

vector_t *get_pid_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kp;
  case 2:
    return &profile.pid.pid_rates[profile.pid.pid_profile].ki;
  case 3:
    return &profile.pid.pid_rates[profile.pid.pid_profile].kd;
  }
  return NULL;
}

vector_t *get_sw_rate_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.rate.silverware.max_rate;
  case 2:
    return &profile.rate.silverware.acro_expo;
  case 3:
    return &profile.rate.silverware.angle_expo;
  }
  return NULL;
}

vector_t *get_bf_rate_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.rate.betaflight.rc_rate;
  case 2:
    return &profile.rate.betaflight.super_rate;
  case 3:
    return &profile.rate.betaflight.expo;
  }
  return NULL;
}

vector_t *get_stick_profile_term(uint8_t term) {
  switch (term) {
  case 1:
    return &profile.pid.stick_rates[profile.pid.stick_profile].accelerator;
  case 2:
    return &profile.pid.stick_rates[profile.pid.stick_profile].transition;
  }
  return NULL;
}

void osd_vector_adjust ( vector_t *pointer, uint8_t rows, uint8_t columns, uint8_t special_case){

	if (osd_select > columns) {
		osd_select = columns;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	if (osd_cursor <= rows){
		uint8_t adjust_tracker = ((osd_cursor-1) * columns) + (osd_select - 1);
		if (special_case == BF_PIDS){
			if (increase_osd_value){
				pointer->axis[osd_select-1] = pointer->axis[osd_select-1] + bf_pids_increments[adjust_tracker];
				increase_osd_value = 0;
				decrease_osd_value = 0;
				osd_menu_phase = 1; //repaint the screen again
			}
			if (decrease_osd_value){
				pointer->axis[osd_select-1] = pointer->axis[osd_select-1] - bf_pids_increments[adjust_tracker];
				increase_osd_value = 0;
				decrease_osd_value = 0;
				osd_menu_phase = 1; //repaint the screen again
			}
		}
		if (special_case == SW_RATES){
			if (increase_osd_value || decrease_osd_value) pointer->axis[osd_select-1] = adjust_rounded_float(pointer->axis[osd_select-1], sw_rates_increments[adjust_tracker]);
		}
		if (special_case == ROUNDED){
			if (increase_osd_value || decrease_osd_value) pointer->axis[osd_select-1] = adjust_rounded_float(pointer->axis[osd_select-1], rounded_increments[adjust_tracker]);
		}
	}
	if (osd_cursor == rows + 1){
		if (osd_select == 1){
		osd_save_exit();
		}
	}
}

void osd_submenu_select (uint8_t *pointer, uint8_t rows, const uint8_t next_menu[]){
	if (osd_select == 1){ //stick was pushed right to select a rate type
		osd_select = 0;	//reset the trigger
		if(osd_cursor <= rows){
			*pointer = osd_cursor-1;	//update profile
			osd_display_phase = next_menu[osd_cursor-1];	//update display phase to the next menu screen
			osd_cursor = 0;	//reset the cursor
			osd_menu_phase = 0;	//clear the screen

		}
	}
}

void osd_select_menu_item(uint8_t rows, const uint8_t menu_map[], uint8_t main_menu) {
	if (osd_select == 1 && flash_feature_1 == 1) //main menu
	{
		osd_select = 0; //reset the trigger
		if (osd_cursor <= rows){
			osd_display_phase = menu_map[osd_cursor-1];
			osd_cursor = 0;
			osd_menu_phase = 0;
		}
		if (main_menu){
			if(osd_cursor == rows + 1) flash_feature_1 = 0; //flag the setup wizard in main menu
			if(osd_cursor == rows + 2) osd_save_exit();		//include save&exit in main menu
		}
	}
}

void osd_enum_adjust(uint8_t *pointer, uint8_t rows, const uint8_t increase_limit[]){
	if(osd_select > 1) {
		osd_select = 1;	//limit osd select variable from accumulating past 1 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	if (osd_cursor <= rows){
		if (osd_select == 1){
			uint8_t i = *pointer;
			if (increase_osd_value && i != increase_limit[osd_cursor-1])  {	//limits need to be 11 for arming, 14 for everything else on flight modes
				i++;
				*pointer = i;
				osd_menu_phase = 1; //repaint the screen again
			}
			if (decrease_osd_value && i != 0)  {	//limit is always 0 for an enum
				i--;
				*pointer = i;
				osd_menu_phase = 1; //repaint the screen again
			}
		}
	increase_osd_value = 0;
	decrease_osd_value = 0;
	}
	if (osd_cursor == rows + 1){
		if (osd_select == 1){
			osd_save_exit();
		}
	}
}

void print_osd_menu_strings (uint8_t string_element_qty, uint8_t active_element_qty, const char element_names[string_element_qty][21], const uint8_t print_position[string_element_qty][2]){
	if (osd_menu_phase >  string_element_qty)
        return;
    if (osd_menu_phase == 0){
    	osd_clear_screen();
        osd_menu_phase++;
        return;
    }
    osd_print(element_names[osd_menu_phase-1], user_select(active_element_qty, string_element_qty), print_position[osd_menu_phase-1][0], print_position[osd_menu_phase-1][1]);
    osd_menu_phase++;
}

void print_osd_adjustable_enums (uint8_t string_element_qty, uint8_t data_element_qty, const char data_to_print[21], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]){
	if (osd_menu_phase <= string_element_qty)
        return;
	if (osd_menu_phase > string_element_qty + data_element_qty)
        return;
	static uint8_t skip_loop = 0;
	if (osd_menu_phase ==  string_element_qty + 1 && skip_loop == 0){	//skip a loop to prevent dma collision with previous print function
		skip_loop++;
		return;
	}
	skip_loop = 0;
	uint8_t index = osd_menu_phase-string_element_qty-1;
	osd_print(data_to_print, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
	osd_menu_phase++;
}

void print_osd_adjustable_vectors(uint8_t menu_type, uint8_t string_element_qty, uint8_t data_element_qty, vector_t *pointer, const uint8_t data_index[data_element_qty][2], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]){
  	if (osd_menu_phase <= string_element_qty)
          return;
  	if (osd_menu_phase > string_element_qty + data_element_qty)
          return;
  	static uint8_t skip_loop = 0;
  	if (osd_menu_phase ==  string_element_qty + 1 && skip_loop == 0){	//skip a loop to prevent dma collision with previous print function
  		skip_loop++;
  		return;
  	}
  	skip_loop = 0;
  	uint8_t index = osd_menu_phase-string_element_qty-1;

  	uint8_t data_buffer[5];
  	if (menu_type == BF_PIDS) fast_fprint(data_buffer, 5, pid_scale(get_pid_term(data_index[index][0])->axis[data_index[index][1]], pid_scale_index[index]), 0);
  	if (menu_type == SW_RATES) ;	//not done yet
  	if (menu_type == ROUNDED) ;		//not done yet
  	osd_print_data(data_buffer, 5, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);

  	osd_menu_phase++;
}



//******************************************************************************************************************************


//******************************************************************************************************************************
// osd main display function

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
    osd_display_reset();
    osd_clear_screen();
    break; //screen has been cleared for this loop - break out of display function

  case 1:                //osd menu is active
    if (flash_feature_1) //setup wizard
    {
    	print_osd_menu_strings(10, 9, main_menu_labels, main_menu_positions);
    	if(osd_menu_phase == 11)osd_select_menu_item(7,main_menu_map, MAIN_MENU);
    } else {
      switch (osd_wizard_phase) {
      case 0:
    	osd_clear_screen();
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

  case 3:		//pids profile menu
	  print_osd_menu_strings(3, 2, pid_profiles_labels, pid_profiles_positions);
	  if (osd_menu_phase == 4)osd_submenu_select (&profile.pid.pid_profile, 2 , pid_submenu_map);
    break;

  case 4:		//pids profiles
	  if(profile.pid.pid_profile == PID_PROFILE_1){
		  print_osd_menu_strings(8, 4, pid_profile1_labels, pid_profile_positions);
	  }else{
		  print_osd_menu_strings(8, 4, pid_profile2_labels, pid_profile_positions);
	  }
	  print_osd_adjustable_vectors(BF_PIDS, 8, 9, get_pid_term(pid_profile_data_index[osd_menu_phase-9][0]), pid_profile_data_index, pid_profile_grid, pid_profile_data_positions);
	  if (osd_menu_phase == 18) osd_vector_adjust(get_pid_term(osd_cursor), 3, 3, BF_PIDS);
    break;

  case 5:		//filters menu
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("FILTERS", INVERT, 11, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("UNDER", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("DEVELOPMENT", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  //osd_select_filters();
    	  break;
      }
    break;

  case 6:		//main rates menu
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("RATES", INVERT, 13, 1);
    	  osd_menu_phase++;
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
    	  osd_submenu_select (&profile.rate.mode, 2 , rates_submenu_map);
    	  break;
      }
    break;

  case 7:		//silverware rates submenu
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
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
          uint8_t osd_roll_rate[5];
          fast_fprint(osd_roll_rate, 5, profile.rate.silverware.max_rate.roll, 0);
          osd_print_data(osd_roll_rate, 5, grid_selection(1, 1), 14, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_pitch_rate[5];
          fast_fprint(osd_pitch_rate, 5, profile.rate.silverware.max_rate.pitch, 0);
          osd_print_data(osd_pitch_rate, 5, grid_selection(2, 1), 19, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_yaw_rate[5];
          fast_fprint(osd_yaw_rate, 5, profile.rate.silverware.max_rate.yaw, 0);
          osd_print_data(osd_yaw_rate, 5, grid_selection(3, 1), 24, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_roll[5];
          fast_fprint(osd_acro_expo_roll, 5, profile.rate.silverware.acro_expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_roll, 5, grid_selection(1, 2), 13, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_pitch[5];
          fast_fprint(osd_acro_expo_pitch, 5, profile.rate.silverware.acro_expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_pitch, 5, grid_selection(2, 2), 18, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_yaw[5];
          fast_fprint(osd_acro_expo_yaw, 5, profile.rate.silverware.acro_expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_yaw, 5, grid_selection(3, 2), 23, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_roll[5];
          fast_fprint(osd_angle_expo_roll, 5, profile.rate.silverware.angle_expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_roll, 5, grid_selection(1, 3), 13, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_pitch[5];
          fast_fprint(osd_angle_expo_pitch, 5, profile.rate.silverware.angle_expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_pitch, 5, grid_selection(2, 3), 18, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_yaw[5];
          fast_fprint(osd_angle_expo_yaw, 5, profile.rate.silverware.angle_expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_yaw, 5, grid_selection(3, 3), 23, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_vector_adjust(get_sw_rate_term(osd_cursor), 3, 3, SW_RATES);
    	  break;
      }
    break;

  case 8:		//betaflight rates submenu
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("BETAFLIGHT RATES", INVERT, 7, 1);
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
          osd_print("RC RATE", user_selection(1, 4), 2, 6);
          osd_menu_phase++;
          break;
      case 6:
    	  osd_print("SUPER RATE", user_selection(2, 4), 2, 7);
          osd_menu_phase++;
          break;
      case 7:
          osd_print("EXPO", user_selection(3, 4), 2, 8);
          osd_menu_phase++;
          break;
      case 8:
          osd_print("SAVE AND EXIT", user_selection(4, 4), 2, 14);
          osd_menu_phase++;
          break;
      case 9:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_roll_rate[5];
          fast_fprint(osd_roll_rate, 5, profile.rate.betaflight.rc_rate.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_rate, 5, grid_selection(1, 1), 13, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_pitch_rate[5];
          fast_fprint(osd_pitch_rate, 5, profile.rate.betaflight.rc_rate.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_rate, 5, grid_selection(2, 1), 18, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_yaw_rate[5];
          fast_fprint(osd_yaw_rate, 5, profile.rate.betaflight.rc_rate.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_rate, 5, grid_selection(3, 1), 23, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_roll[5];
          fast_fprint(osd_acro_expo_roll, 5, profile.rate.betaflight.super_rate.roll + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_roll, 5, grid_selection(1, 2), 13, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_pitch[5];
          fast_fprint(osd_acro_expo_pitch, 5, profile.rate.betaflight.super_rate.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_pitch, 5, grid_selection(2, 2), 18, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_yaw[5];
          fast_fprint(osd_acro_expo_yaw, 5, profile.rate.betaflight.super_rate.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_yaw, 5, grid_selection(3, 2), 23, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_roll[5];
          fast_fprint(osd_angle_expo_roll, 5, profile.rate.betaflight.expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_roll, 5, grid_selection(1, 3), 13, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_pitch[5];
          fast_fprint(osd_angle_expo_pitch, 5, profile.rate.betaflight.expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_pitch, 5, grid_selection(2, 3), 18, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_yaw[5];
          fast_fprint(osd_angle_expo_yaw, 5, profile.rate.betaflight.expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_yaw, 5, grid_selection(3, 3), 23, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_vector_adjust(get_bf_rate_term(osd_cursor), 3, 3, ROUNDED);
    	  break;
      }
    break;

  case 9:		//flight modes menu
	  print_osd_menu_strings(12, 11, flight_modes_labels, flight_modes_positions);
	  print_osd_adjustable_enums (12, 10, get_aux_status(profile.channel.aux[flight_modes_aux_items[osd_menu_phase-13]]), flight_modes_grid, flight_modes_data_positions);
	  if (osd_menu_phase==23)osd_enum_adjust(&profile.channel.aux[flight_modes_aux_items[osd_cursor-1]], 10, flight_modes_aux_limits);
    break;

  case 10:		//osd elements menu
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("OSD ELEMENTS", INVERT, 9, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("UNDER", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("DEVELOPMENT", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  //osd_select_elements();
    	  break;
      }
    break;

  case 11:		//vtx
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("VTX CONTROLS", INVERT, 9, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("UNDER", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("DEVELOPMENT", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  //osd_select_vtx();
    	  break;
      }
    break;

  case 12:		//special features
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
          osd_print("SPECIAL FEATURES", INVERT, 7, 1);
          osd_menu_phase++;
          break;
      case 2:
          osd_print("STICK BOOST", user_selection(1, 1), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
    	  osd_select_menu_item(1,special_features_map, SUB_MENU);
    	  break;
      }
      break;

  case 13:		//stick accelerator profiles
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
          osd_print("STICK BOOST PROFILES", INVERT, 5, 1);
          osd_menu_phase++;
          break;
      case 2:
          osd_print("AUX OFF PROFILE 1", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("AUX ON  PROFILE 2", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  osd_submenu_select (&profile.pid.stick_profile, 2 , stickboost_submenu_map);
    	  break;
      }
      break;

  case 14:		//stick boost profiles
      switch (osd_menu_phase) {
      case 0:
    	  osd_clear_screen();
          osd_menu_phase++;
          break;
      case 1:
    	  if(profile.pid.stick_profile == STICK_PROFILE_1){
    	  osd_print("BOOST PROFILE 1", INVERT, 8, 1);
    	  osd_menu_phase++;
    	  }else{
          osd_print("BOOST PROFILE 2", INVERT, 8, 1);
          osd_menu_phase++;
    	  }
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
          osd_print("STICK", user_selection(1, 3), 2, 5);
          osd_menu_phase++;
          break;
      case 6:
          osd_print("ACCELERATOR", user_selection(1, 3), 2, 6);
          osd_menu_phase++;
          break;
      case 7:
          osd_print("STICK", user_selection(2, 3), 2, 7);
          osd_menu_phase++;
          break;
      case 8:
    	  osd_print("TRANSITION", user_selection(2, 3), 2, 8);
          osd_menu_phase++;
          break;
      case 9:
          osd_print("SAVE AND EXIT", user_selection(3, 3), 2, 14);
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_roll_accel[5];
          fast_fprint(osd_roll_accel, 5, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_accel, 5, grid_selection(1, 1), 13, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_pitch_accel[5];
          fast_fprint(osd_pitch_accel, 5, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_accel, 5, grid_selection(2, 1), 18, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_yaw_accel[5];
          fast_fprint(osd_yaw_accel, 5, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_accel, 5, grid_selection(3, 1), 23, 6);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_roll_trans[5];
          fast_fprint(osd_roll_trans, 5, profile.pid.stick_rates[profile.pid.stick_profile].transition.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_trans, 5, grid_selection(1, 2), 13, 8);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_pitch_trans[5];
          fast_fprint(osd_pitch_trans, 5, profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_trans, 5, grid_selection(2, 2), 18, 8);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_yaw_trans[5];
          fast_fprint(osd_yaw_trans, 5, profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_trans, 5, grid_selection(3, 2), 23, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  osd_vector_adjust(get_stick_profile_term(osd_cursor), 2, 3, ROUNDED);
    	  break;
      }
    break;

  }

} //end osd_display()
//******************************************************************************************************************************
#endif
