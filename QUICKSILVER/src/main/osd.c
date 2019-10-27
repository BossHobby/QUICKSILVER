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
#include "debug.h"

#ifdef ENABLE_OSD

void osd_init(void) {
  spi_max7456_init(); //init spi
  max7456_init();     //init the max chip
  osd_intro();        //print the splash screen
}

/*screen elements characteristics written like registers in a 32bit binany number
except callsign which will take 6 addresses.  callsign bit 1 will be enable/disable,
bit 2 will be text/invert, and the remaining 5 addresses will be 4 characters each.  6 total addresses
will allow callsign text to fill the whole screen across
BIT
0			-		0 is display element inactive , 1 is display element active
1			-		0 is TEXT, 1 is INVERT
2:6		-		the X screen position (column)
7:10	-		the Y screen position	(row)
11:15	-		not currently used
16:31	-		available for two binary ascii characters but not currently used
*/

//Flash Variables - 32bit					# of osd elements and flash memory start position in defines.h
extern unsigned long osd_element[OSD_NUMBER_ELEMENTS];
/*
elements 0-5 - Call Sign
element 6 Fuel Gauge volts
element 7 Filtered Volts
element 8 Exact Volts
***********************
***********************
element 9 Flight Mode
element 10 Time
element 11 RSSI
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

unsigned long *flight_mode = (osd_element + 9);
unsigned long *rssi = (osd_element + 10);
unsigned long *stopwatch = (osd_element + 11);
unsigned long *arm_disarm = (osd_element + 12);
unsigned long *osd_throttle = (osd_element + 13);
unsigned long *osd_vtx = (osd_element + 14);

//screen element register decoding functions
uint8_t decode_active_element(uint32_t element){
	return (element & 0x01);
}

uint8_t decode_attribute(uint32_t element) { //shifting right one bit and comparing the new bottom bit to the key above
  uint8_t decoded_element = ((element >> 1) & 0x01);
  if (decoded_element == 0x01)
    return INVERT;
  else
    return TEXT;
}

uint8_t decode_positionx(uint32_t element) { //shift 2 bits and grab the bottom 5
  return ((element >> 2) & 0x1F);
}

uint8_t decode_positiony(uint32_t element) { //shift 7 bits and grab the bottom 4
  return ((element >> 7) & 0x0F);
}

#define ACTIVE 0
#define ATTRIBUTE 1
#define POSITIONX 2
#define POSITIONY 3

uint8_t osd_decode(uint32_t element, uint8_t status){
	switch (status){
	case 0:
		return (element & 0x01);
		break;
	case 1:
		if (((element >> 1) & 0x01) == 0x01) return INVERT;
		else return TEXT;
		break;
	case 2:
		return ((element >> 2) & 0x1F);
		break;
	case 3:
		return ((element >> 7) & 0x0F);
		break;
	}
	return 0;
}


const char* get_position_string (int input){
	static char* respond[] = {" 0", " 1", " 2", " 3", " 4", " 5", " 6", " 7", " 8", " 9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "ERROR"};
	return respond[input];
}


const char* get_decode_element_string (uint32_t input , uint8_t status){
	switch (status){
	case 0:		//ACTIVE
		if (osd_decode(input, status)) return "ACTIVE  ";
		else return "INACTIVE";
		break;
	case 1:		//ATTRIBUTE
		if (osd_decode(input, status) == INVERT) return "INVERT";
		else return "NORMAL";
		break;
	case 2:
		return get_position_string(osd_decode(input, status));
		break;
	case 3:
		return get_position_string(osd_decode(input, status));
		break;
	}
	return 0;
}



//******************************************************************************************************************************


//******************************************************************************************************************************
// case & state variables for switch logic and profile adjustments
debug_type debug;
extern int flash_feature_1; //currently used for auto entry into wizard menu
extern profile_t profile;
uint8_t osd_display_phase = 2;
uint8_t last_display_phase;
uint8_t osd_wizard_phase = 0;
uint8_t osd_menu_phase = 0;
uint8_t osd_display_element = 0;
uint8_t display_trigger = 0;
uint8_t last_lowbatt_state = 2;
uint8_t last_lowbatt_state2 = 2;
uint8_t last_lowbatt_state3 = 2;
uint8_t osd_cursor;
uint8_t last_osd_cursor[6];
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
const uint8_t pid_profile_data_index[9][2] = { {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2} };
const uint8_t pid_profile_grid[9][2] = { {1, 1},{2, 1},{3, 1},{1, 2},{2, 2},{3, 2},{1, 3},{2, 3},{3, 3} };
const uint8_t pid_profile_data_positions[9][2] = { {10, 6},{16, 6},{22, 6},{10, 7},{16, 7},{22, 7},{10, 8},{16, 8},{22, 8} };
const float pid_profile_adjust_limits[9][2] = { {0.0015924, 1.0}, {0.0015924, 1.0}, {0.0031847, 1.0}, {0.02, 3.0}, {0.02, 3.0}, {0.02, 3.0}, {0.0083333, 1.0}, {0.0083333, 1.0}, {0.0083333, 1.0} };
const uint8_t pid_scale_index[9] = {0, 0, 1, 2, 2, 2, 3, 3, 3};

//filters submenu map
const char filter_temp_labels[3][21] = { {"FILTERS"},{"UNDER"},{"DEVELOPMENT"} };
const uint8_t filter_temp_positions[3][2] = { {11, 1}, {7, 4}, {7,5} };

//rates submenu map
const char rates_profile_labels[3][21] = { {"RATES"}, {"SILVERWARE"}, {"BETAFLIGHT"} };
const uint8_t rates_profile_positions[3][2] = { {13, 1},{7, 4},{7, 5} };
const uint8_t rates_submenu_map[] = {7, 8};

//silverware rates map
const char sw_rates_labels[8][21] = { {"SILVERWARE RATES"},{"ROLL"},{"PITCH"},{"YAW"},{"RATE"},{"ACRO EXPO"},{"ANGLE EXPO"},{"SAVE AND EXIT"} };
const uint8_t sw_rates_positions[8][2] = { {7, 1},{14, 4},{19, 4},{25, 4},{2, 6},{2, 7},{2, 8},{2, 14} };
const uint8_t sw_rates_data_index[9][2] = { {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2} };
const uint8_t sw_rates_grid[9][2] = { {1, 1},{2, 1},{3, 1},{1, 2},{2, 2},{3, 2},{1, 3},{2, 3},{3, 3} };
const uint8_t sw_rates_data_positions[9][2] = { {14, 6},{19, 6},{24, 6},{13, 7},{18, 7},{23, 7},{13, 8},{18, 8},{23, 8} };
const float sw_rates_adjust_limits[9][2] = { {0, 1800.0}, {0, 1800.0}, {0, 1800.0}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99} };

//betaflight rates map
const char bf_rates_labels[8][21] = { {"BETAFLIGHT RATES"},{"ROLL"},{"PITCH"},{"YAW"},{"RC RATE"},{"SUPER RATE"},{"EXPO"},{"SAVE AND EXIT"} };
const uint8_t bf_rates_positions[8][2] = { {7, 1},{14, 4},{19, 4},{25, 4},{2, 6},{2, 7},{2, 8},{2, 14} };
const uint8_t bf_rates_data_index[9][2] = { {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2} };
const uint8_t bf_rates_grid[9][2] = { {1, 1},{2, 1},{3, 1},{1, 2},{2, 2},{3, 2},{1, 3},{2, 3},{3, 3} };
const uint8_t bf_rates_data_positions[9][2] = { {13, 6},{18, 6},{23, 6},{13, 7},{18, 7},{23, 7},{13, 8},{18, 8},{23, 8} };
const float bf_rates_adjust_limits[9][2] = { {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 0.99}, {0, 0.99}, {0, 0.99} };

//flight modes map
const char flight_modes_labels[12][21] = {"FLIGHT MODES","ARMING","IDLE UP","LEVELMODE","RACEMODE","HORIZON","STICK BOOST","HIGH RATES","BUZZER","TURTLE","MOTOR TEST","SAVE AND EXIT"};
const uint8_t flight_modes_positions[12][2] = { {9, 1},{4, 2},{4, 3},{4, 4},{4, 5},{4, 6},{4, 7},{4, 8},{4, 9},{4, 10},{4, 11},{4, 14} };
const uint8_t flight_modes_data_positions[10][2] = { {17, 2}, {17, 3}, {17, 4}, {17, 5}, {17, 6}, {17, 7}, {17, 8}, {17, 9}, {17, 10}, {17, 11} };
const uint8_t flight_modes_aux_limits[] = {11, 14, 14, 14, 14, 14, 14, 14, 14, 14};	//from aux_channel_t
const uint8_t flight_modes_aux_items[] = {0, 1, 2, 3, 4, 5, 7, 9, 10, 12};			//from aux_function_t
const uint8_t flight_modes_grid[10][2] = { {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10} };

//osd elements submenu map
const char osd_elements_menu_labels[5][21] = { {"OSD ELEMENTS"},{"ADD OR REMOVE"},{"EDIT POSITIONS"},{"EDIT TEXT STYLE"},{"EDIT CALLSIGN"} };
const uint8_t osd_elements_menu_positions[5][2] = { {9, 1}, {7, 4}, {7, 5}, {7, 6}, {7, 7} };
const uint8_t osd_elements_map[] = {15, 16, 17, 18};

//osd element add/remove & text/invert submenu map
const char osd_display_labels[12][21] = { {"OSD DISPLAY ITEMS"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"EXACT VOLTS"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"} };
const uint8_t osd_display_positions[12][2] = { {6,1}, {4,2}, {4,3}, {4,4}, {4,5}, {4,6}, {4,7}, {4,8}, {4,9}, {4,10}, {4,11}, {4,14} };
const uint8_t osd_display_data_positions[10][2] = { {20, 2}, {20, 3}, {20, 4}, {20, 5}, {20, 6}, {20, 7}, {20, 8}, {20, 9}, {20, 10}, {20, 11} };
const uint8_t osd_display_grid[10][2] = { {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10} };
const uint8_t osd_elements_active_items[] = {0, 6, 7, 8, 9, 10, 11, 12, 13, 14};

//osd positions submenu map
const char osd_position_labels[14][21] = { {"OSD POSITIONS"},{"ADJ X"}, {"ADJ Y"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"EXACT VOLTS"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"} };
const uint8_t osd_position_adjust_positions[14][2] = { {1,1}, {18,1}, {24,1}, {3,2}, {3,3}, {3,4}, {3,5}, {3,6}, {3,7}, {3,8}, {3,9}, {3,10}, {3,11}, {3,14} };
const uint8_t osd_position_grid[20][2] = { {1, 1}, {2, 1}, {1, 2}, {2, 2}, {1, 3}, {2, 3}, {1, 4}, {2, 4}, {1, 5}, {2, 5}, {1, 6}, {2, 6}, {1, 7}, {2, 7}, {1, 8}, {2, 8}, {1, 9}, {2, 9}, {1, 10}, {2, 10} };
const uint8_t osd_position_data_positions[20][2] = { {20, 2}, {26, 2}, {20, 3}, {26, 3}, {20, 4}, {26, 4}, {20, 5}, {26, 5}, {20, 6}, {26, 6}, {20, 7}, {26, 7}, {20, 8}, {26, 8}, {20, 9}, {26, 9}, {20, 10}, {26, 10}, {20, 11}, {26, 11} };
const uint8_t osd_position_active_items[] = {0, 0, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14};
const uint8_t osd_position_index[20] = {2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3};

//osd text style submenu map
const char osd_text_style[12][21] = { {"OSD TEXT STYLE"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"EXACT VOLTS"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"} };
const uint8_t osd_text_style_positions[12][2] = { {8,1}, {4,2}, {4,3}, {4,4}, {4,5}, {4,6}, {4,7}, {4,8}, {4,9}, {4,10}, {4,11}, {4,14} };

//osd callsign edit submenu map
const char osd_callsign_edit_labels[23][21] = { {"CALLSIGN"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"-"},{"EDIT:"},{"SAVE AND EXIT"} };
const uint8_t osd_callsign_edit_positions[23][2] = { {11, 1}, {8, 6}, {9, 6}, {10, 6}, {11, 6}, {12, 6}, {13, 6}, {14, 6}, {15, 6}, {16, 6}, {17, 6}, {18, 6}, {19, 6}, {20, 6}, {21, 6}, {22, 6}, {23, 6}, {24, 6}, {25, 6}, {26, 6}, {27, 6}, {1, 5}, {1, 7} };
const uint8_t osd_callsign_grid [20][2] = { {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {8, 1}, {9, 1}, {10, 1}, {11, 1}, {12, 1}, {13, 1}, {14, 1}, {15, 1}, {16, 1}, {17, 1}, {18, 1}, {19, 1}, {20, 1} };
const uint8_t osd_callsign_edit_data_positions[20][2] = { {8, 5}, {9, 5}, {10, 5}, {11, 5}, {12, 5}, {13, 5}, {14, 5}, {15, 5}, {16, 5}, {17, 5}, {18, 5}, {19, 5}, {20, 5}, {21, 5}, {22, 5}, {23, 5}, {24, 5}, {25, 5}, {26, 5}, {27, 5} };
const uint8_t callsign_shift_index[20][2] = { {1, 0} , {1, 8} , {1, 16} , {1, 24} , {2, 0} , {2, 8} , {2, 16} , {2, 24} , {3, 0} , {3, 8} , {3, 16} , {3, 24} , {4, 0} , {4, 8} , {4, 16} , {4, 24} , {5, 0} , {5, 8} , {5, 16} , {5, 24} };

//vtx submenu map
const char vtx_temp_labels[3][21] = { {"VTX CONTROLS"},{"UNDER"},{"DEVELOPMENT"} };
const uint8_t vtx_temp_positions[3][2] = { {9, 1}, {7, 4}, {7,5} };

//special features menu map
const char special_features_labels[6][21] = { {"SPECIAL FEATURES"},{"STICK BOOST"},{"LOW BATTERY"},{"LEVEL MODE"},{"TORQUE BOOST"},{"DIGITAL IDLE"} };
const uint8_t special_features_positions[6][2] = { {7, 1}, {7, 4}, {7, 5}, {7, 6}, {7, 7}, {7, 8} };
const uint8_t special_features_map[] = {13, 19, 20, 21, 22};					//case numbers for {stickboost}, etc ...adding more soon

//stick boost submenu map
const char stickboost_labels[3][21] = { {"STICK BOOST PROFILES"},{"AUX OFF PROFILE 1"},{"AUX ON  PROFILE 2"} };
const uint8_t stickboost_profile_positions[3][2] = { {5, 1},{7, 4},{7, 5} };
const uint8_t stickboost_submenu_map[] = {14, 14};

//stick boost map
const char stickboost1_labels[7][21] = { {"BOOST PROFILE 1"},{"ROLL"},{"PITCH"},{"YAW"},{"ACCELERATOR"},{"TRANSITION"},{"SAVE AND EXIT"} };
const char stickboost2_labels[7][21] = { {"BOOST PROFILE 2"},{"ROLL"},{"PITCH"},{"YAW"},{"ACCELERATOR"},{"TRANSITION"},{"SAVE AND EXIT"} };
const uint8_t stickboost_positions[7][2] = { {8, 1},{14, 4},{19, 4},{25, 4},{2, 6},{2, 8},{2, 14} };
const uint8_t stickboost_data_index[6][2] = { {1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2} };
const uint8_t stickboost_grid[6][2] = { {1, 1},{2, 1},{3, 1},{1, 2},{2, 2},{3, 2} };
const uint8_t stickboost_data_positions[9][2] = { {13, 6},{18, 6},{23, 6},{13, 8},{18, 8},{23, 8} };
const float stickboost_adjust_limits[6][2] = { {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 1.0}, {0, 1.0}, {0, 1.0} };

//low battery map
float *low_batt_ptr[1] = {&profile.voltage.vbattlow};
const char lowbatt_labels[3][21] = { {"LOW BATTERY"},{"VOLTS/CELL ALERT"},{"SAVE AND EXIT"} };
const uint8_t lowbatt_positions[3][2] = { {10, 1},{4, 5},{4, 14} };
const uint8_t lowbatt_grid [1][2] = {{1, 1}};
const uint8_t lowbatt_data_positions[1][2] = { {21, 5} };
const float lowbatt_adjust_limits[1][2] = { {0, 4.2} };

//levelmode submenu map
const char level_submenu_labels[3][21] = { {"LEVEL MODE"},{"MAX ANGLE"},{"LEVEL STRENGTH"} };
const uint8_t level_submenu_positions[3][2] = { {10, 1}, {7, 4}, {7, 5} };
const uint8_t level_submenu_map[] = {23, 24};

//levelmode maxangle map
float *level_maxangle_ptr[1] = {&profile.rate.level_max_angle};
const char maxangle_labels[3][21] = { {"LEVEL MODE"},{"MAX ANGLE DEGREES"},{"SAVE AND EXIT"} };
const uint8_t maxangle_positions[3][2] = { {10, 1},{1, 5},{1, 14} };
const uint8_t maxangle_grid [6][2] = {{1, 1}};
const uint8_t maxangle_data_positions[1][2] = { {19, 5} };
const float maxangle_adjust_limits[1][2] = { {0, 85.0} };

//levelmode pid map
float *level_pid_ptr[4] = {&profile.pid.small_angle.kp, &profile.pid.small_angle.kd, &profile.pid.big_angle.kp, &profile.pid.big_angle.kd};
const char levelmode_labels[6][21] = { {"LEVEL MODE"},{"KP"},{"KD"},{"SM ANGLE STRENGTH"},{"LRG ANGLE STRENGTH"},{"SAVE AND EXIT"} };
const uint8_t levelmode_positions[6][2] = { {10, 1},{21, 5},{26, 5},{1, 6},{1, 7},{1, 14} };
const uint8_t levelmode_grid [4][2] = {{1, 1}, {2, 1}, {1, 2}, {2, 2}};
const uint8_t levelmode_data_positions[4][2] = { {19, 6}, {24, 6}, {19, 7}, {24, 7} };
const float levelmode_adjust_limits[4][2] = { {0, 20.0}, {0, 10.0}, {0, 20.0}, {0, 10.0} };

//torque boost map
float *torqueboost_ptr[1] = {&profile.motor.torque_boost};
const char torqueboost_labels[3][21] = { {"TORQUE BOOST"},{"MOTOR TORQUE BOOST"},{"SAVE AND EXIT"} };
const uint8_t torqueboost_positions[3][2] = { {9, 1},{4, 5},{4, 14} };
const uint8_t torqueboost_grid [1][2] = {{1, 1}};
const uint8_t torqueboost_data_positions[1][2] = { {22, 5} };
const float torqueboost_adjust_limits[1][2] = { {0, 3.0} };

//digital idle map
float *motoridle_ptr[1] = {&profile.motor.digital_idle};
const char motoridle_labels[3][21] = { {"DIGITAL IDLE"},{"MOTOR IDLE %"},{"SAVE AND EXIT"} };
const uint8_t motoridle_positions[3][2] = { {9, 1},{4, 5},{4, 14} };
const uint8_t motoridle_grid [1][2] = {{1, 1}};
const uint8_t motoridle_data_positions[1][2] = { {17, 5} };
const float motoridle_adjust_limits[1][2] = { {0, 25.0} };

//******************************************************************************************************************************


//******************************************************************************************************************************
// osd utility functions

void osd_display_reset(void) {
  osd_wizard_phase = 0;    //reset the wizard
  osd_menu_phase = 0;      //reset menu to to main menu
  osd_display_phase = 2;   //jump to regular osd display next loop
  osd_display_element = 0; //start with first screen element
  last_lowbatt_state = 2;  //reset last lowbatt comparator
  last_lowbatt_state2 = 2;
  last_lowbatt_state3 = 2;
}

void osd_save_exit(void){
	osd_select = 0;
    osd_cursor = 0;
    for (uint8_t i = 0; i < 6; i++){
    	last_osd_cursor[i] = 0;
    }
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
	return respond[input];
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


void osd_vector_adjust ( vector_t *pointer, uint8_t rows, uint8_t columns, uint8_t special_case, const float adjust_limit[rows*columns][2]){
	if (osd_select > columns) {
		osd_select = columns;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	if (osd_cursor <= rows){
		uint8_t adjust_tracker = ((osd_cursor-1) * columns) + (osd_select - 1);
		if ((increase_osd_value && pointer->axis[osd_select-1] < adjust_limit[adjust_tracker][1]) || (decrease_osd_value  && pointer->axis[osd_select-1] > adjust_limit[adjust_tracker][0])){
			if (special_case == BF_PIDS){
				if (increase_osd_value) pointer->axis[osd_select-1] = pointer->axis[osd_select-1] + bf_pids_increments[adjust_tracker];
				if (decrease_osd_value) pointer->axis[osd_select-1] = pointer->axis[osd_select-1] - bf_pids_increments[adjust_tracker];
				osd_menu_phase = 1; //repaint the screen again
			}
			if (special_case == SW_RATES)
				pointer->axis[osd_select-1] = adjust_rounded_float(pointer->axis[osd_select-1], sw_rates_increments[adjust_tracker]);
			if (special_case == ROUNDED)
				pointer->axis[osd_select-1] = adjust_rounded_float(pointer->axis[osd_select-1], rounded_increments[adjust_tracker]);
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

void osd_float_adjust ( float *pointer[],  uint8_t rows, uint8_t columns, const float adjust_limit[rows*columns][2], float adjust_amount){
	if (osd_select > columns) {
		osd_select = columns;
		osd_menu_phase = 1; //repaint the screen again
	}
	if (osd_cursor <= rows){
		uint8_t adjust_tracker = ((osd_cursor-1) * columns) + (osd_select - 1);
		if ((increase_osd_value && *pointer[adjust_tracker] < adjust_limit[adjust_tracker][1]) || (decrease_osd_value  && *pointer[adjust_tracker] > adjust_limit[adjust_tracker][0])){
			*pointer[adjust_tracker] = adjust_rounded_float(*pointer[adjust_tracker], adjust_amount);
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

uint8_t last_cursor_array_stuffer(uint8_t cursor, uint8_t add_new){  	//where add_new can be either STORE_VALUE or RETURN_VALUE
	if (add_new){
		for (int i = 5; i >= 1; i--){	//shift all the values to the right one array position
			last_osd_cursor[i] = last_osd_cursor[i-1];
		}
		last_osd_cursor[0] = cursor;	//add the most recent value to slot 0
		return 0;
	}else{
		uint8_t next_cursor_value = last_osd_cursor[0];	//remove slot 0 for return and left shift the rest over
		for (int i = 1; i <= 5; i++){
			last_osd_cursor[i-1] = last_osd_cursor[i];
		}
	return next_cursor_value;
	}
}

void osd_submenu_select (uint8_t *pointer, uint8_t rows, const uint8_t next_menu[]){
	if (osd_select == 1){ //stick was pushed right to select a next menu
		osd_select = 0;	//reset the trigger
		last_cursor_array_stuffer(osd_cursor, STORE_VALUE);
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
		last_cursor_array_stuffer(osd_cursor, STORE_VALUE);
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

uint32_t get_callsign_bitmask(uint8_t input){
	if(input == 8)  return 0xFF;
	if(input == 16) return 0xFFFF;
	if(input == 24) return 0xFFFFFF;
	return 0;
}

void osd_encoded_adjust_callsign(void){
	if(osd_select > 20) {
		osd_select = 20;	//limit osd select variable from accumulating past 1 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	if (increase_osd_value){
		osd_element[callsign_shift_index[osd_select-1][0]] = (((osd_element[callsign_shift_index[osd_select-1][0]] >> callsign_shift_index[osd_select-1][1]) + 1) << callsign_shift_index[osd_select-1][1]) + (osd_element[callsign_shift_index[osd_select-1][0]] & get_callsign_bitmask(callsign_shift_index[osd_select-1][1]));
		osd_menu_phase = 1; //repaint the screen again
	}
	if (decrease_osd_value){
		osd_element[callsign_shift_index[osd_select-1][0]] = (((osd_element[callsign_shift_index[osd_select-1][0]] >> callsign_shift_index[osd_select-1][1]) - 1) << callsign_shift_index[osd_select-1][1]) + (osd_element[callsign_shift_index[osd_select-1][0]] & get_callsign_bitmask(callsign_shift_index[osd_select-1][1]));
		osd_menu_phase = 1; //repaint the screen again
	}
	increase_osd_value = 0;
	decrease_osd_value = 0;
	if (osd_cursor == 2){
		if (osd_select == 1){
			osd_save_exit();
		}
	}
}

void print_osd_callsign_adjustable (uint8_t string_element_qty, uint8_t data_element_qty, const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2]){
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
	uint8_t character[] = {(osd_element[callsign_shift_index[index][0]] >> callsign_shift_index[index][1]) & 0xFF};
	osd_print_data( character, 1, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
	osd_menu_phase++;
}

uint8_t print_osd_callsign(void){
	static uint8_t index = 0;
	static uint8_t callsign_length = 0;
	if (index == 0){
		for (uint8_t i = 19; i >= 0; i--){
			uint8_t last_user_input = (osd_element[callsign_shift_index[i][0]] >> callsign_shift_index[i][1]) & 0xFF;
			if (last_user_input != 0x3F){
				callsign_length = i + 1;
				index++;
				return 0;
			}
		}
		callsign_length = 0;	//for loop found callsign is all spaces so set length to 0 and return
		index = 0;
		return 1;
	}

	if(index <= callsign_length && callsign_length > 0){
		uint8_t character[] = {(osd_element[callsign_shift_index[index-1][0]] >> callsign_shift_index[index-1][1]) & 0xFF};
		osd_print_data( character, 1, decode_attribute(*callsign1), decode_positionx(*callsign1) + index-1, decode_positiony(*callsign1));
		index++;
		return 0;
	}
	index = 0;
	return 1;
}

uint8_t print_osd_flightmode(void){
	const char flightmode_labels[5][21] = { {"   ACRO   "},{"  LEVEL   "},{" RACEMODE "},{" HORIZON  "},{"RM HORIZON"} };
	static uint8_t index = 0;
	uint8_t flightmode;
	if (rx_aux_on(AUX_LEVELMODE)){
		if (rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))flightmode = 4;
		if (!rx_aux_on(AUX_RACEMODE) && rx_aux_on(AUX_HORIZON))flightmode = 3;
		if (rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON))flightmode = 2;
		if (!rx_aux_on(AUX_RACEMODE) && !rx_aux_on(AUX_HORIZON))flightmode = 1;
	}else{
		flightmode = 0;
	}
	if(index < 10){
		uint8_t character[] = {flightmode_labels[flightmode][index]};
		osd_print_data( character, 1, decode_attribute(*flight_mode), decode_positionx(*flight_mode) + index, decode_positiony(*flight_mode));
		index++;
		return 0;
	}
	index = 0;
	return 1;
}

uint8_t print_osd_system_status(void){
	const char system_status_labels[9][21] = { {"               "},{" **DISARMED**  "},{"  **ARMED**    "},{" STICK BOOST 1 "},{" STICK BOOST 2 "},{" **FAILSAFE**  "},{"THROTTLE SAFETY"},{" ARMING SAFETY "},{"**LOW BATTERY**"} };
	extern int armed_state;
	static uint8_t last_armed_state;
	static uint8_t armed_state_printing;
	static uint8_t last_aux_state;
	static uint8_t aux_state_printing;
	extern int failsafe;
	static uint8_t last_failsafe_state = 1;
	static uint8_t failsafe_state_printing;
	extern uint8_t throttle_safety;
	static uint8_t last_throttle_safety_state;
	static uint8_t throttle_safety_state_printing;
	extern int binding_while_armed;
	static uint8_t last_binding_while_armed_state;
	static uint8_t binding_while_armed_state_printing;
	extern int lowbatt;
	static uint8_t last_lowbatt_state;
	static uint8_t lowbatt_state_printing;
	static uint8_t index = 0;
	static uint8_t counter;
	if(armed_state != last_armed_state || armed_state_printing){
		last_armed_state = armed_state;
		if (armed_state_printing == 2){
			counter++;
			if (counter > 25){
				uint8_t character[] = {system_status_labels[0][index]};
				osd_print_data( character, 1, decode_attribute(*arm_disarm), decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
				index++;
				if (index < 15){
					return 0;
				}else{
					armed_state_printing = 0;
					counter = 0;
					index = 0;
					return 1;
				}
			}
		}
		if (armed_state == 0) {
			uint8_t character[] = {system_status_labels[1][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				armed_state_printing = 1;
				return 0;
			}else{
				index = 0;
				armed_state_printing = 2;
				return 1;
			}
		}
		if (armed_state == 1) {
			uint8_t character[] = {system_status_labels[2][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				armed_state_printing = 1;
				return 0;
			}else{
				index = 0;
				armed_state_printing = 2;
				return 1;
			}
		}
	}
	if(rx_aux_on(AUX_STICK_BOOST_PROFILE) != last_aux_state || aux_state_printing){
		last_aux_state = rx_aux_on(AUX_STICK_BOOST_PROFILE);
		if (aux_state_printing == 2){
			counter++;
			if (counter > 25){
				uint8_t character[] = {system_status_labels[0][index]};
				osd_print_data( character, 1, decode_attribute(*arm_disarm), decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
				index++;
				if (index < 15){
					return 0;
				}else{
					aux_state_printing = 0;
					counter = 0;
					index = 0;
					return 1;
				}
			}
		}
		if (rx_aux_on(AUX_STICK_BOOST_PROFILE) == 0) {
			uint8_t character[] = {system_status_labels[3][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				aux_state_printing = 1;
				return 0;
			}else{
				index = 0;
				aux_state_printing = 2;
				return 1;
			}
		}
		if (rx_aux_on(AUX_STICK_BOOST_PROFILE) == 1) {
			uint8_t character[] = {system_status_labels[4][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				aux_state_printing = 1;
				return 0;
			}else{
				index = 0;
				aux_state_printing = 2;
				return 1;
			}
		}
	}
	if(failsafe != last_failsafe_state || failsafe_state_printing){
		last_failsafe_state = failsafe;
		if (failsafe == 0) {
			uint8_t character[] = {system_status_labels[0][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				failsafe_state_printing = 1;
				return 0;
			}else{
				index = 0;
				failsafe_state_printing = 0;
				return 1;
			}
		}
		if (failsafe == 1) {
			uint8_t character[] = {system_status_labels[5][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				failsafe_state_printing = 1;
				return 0;
			}else{
				index = 0;
				failsafe_state_printing = 0;
				return 1;
			}
		}
	}
	if((binding_while_armed != last_binding_while_armed_state && !failsafe) ||binding_while_armed_state_printing){
		last_binding_while_armed_state = binding_while_armed;
		if (binding_while_armed == 0) {
			uint8_t character[] = {system_status_labels[0][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				binding_while_armed_state_printing = 1;
				return 0;
			}else{
				index = 0;
				binding_while_armed_state_printing = 0;
				return 1;
			}
		}
		if (binding_while_armed == 1) {
			uint8_t character[] = {system_status_labels[7][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				binding_while_armed_state_printing = 1;
				return 0;
			}else{
				index = 0;
				binding_while_armed_state_printing = 0;
				return 1;
			}
		}
	}
	if((throttle_safety != last_throttle_safety_state && !binding_while_armed) || throttle_safety_state_printing){
		last_throttle_safety_state = throttle_safety;
		if (throttle_safety == 0) {
			uint8_t character[] = {system_status_labels[0][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				throttle_safety_state_printing = 1;
				return 0;
			}else{
				index = 0;
				throttle_safety_state_printing = 0;
				return 1;
			}
		}
		if (throttle_safety == 1) {
			uint8_t character[] = {system_status_labels[6][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				throttle_safety_state_printing = 1;
				return 0;
			}else{
				index = 0;
				throttle_safety_state_printing = 0;
				return 1;
			}
		}
	}
	if((lowbatt != last_lowbatt_state && !binding_while_armed && !throttle_safety && !failsafe) || lowbatt_state_printing){
		last_lowbatt_state = lowbatt;
		if (lowbatt == 0) {
			uint8_t character[] = {system_status_labels[0][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				lowbatt_state_printing = 1;
				return 0;
			}else{
				index = 0;
				lowbatt_state_printing = 0;
				return 1;
			}
		}
		if (lowbatt == 1) {
			uint8_t character[] = {system_status_labels[8][index]};
			osd_print_data( character, 1, decode_attribute(*arm_disarm) | BLINK, decode_positionx(*arm_disarm) + index, decode_positiony(*arm_disarm));
			index++;
			if (index < 15){
				lowbatt_state_printing = 1;
				return 0;
			}else{
				index = 0;
				lowbatt_state_printing = 0;
				return 1;
			}
		}
	}
	return 1;
}

void osd_encoded_adjust(uint32_t *pointer, uint8_t rows, uint8_t columns, uint8_t status){
	if(osd_select > columns) {
		osd_select = columns;	//limit osd select variable from accumulating past 1 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	if (osd_cursor <= rows){
		switch (status){
		case 0:	//adjust active or inactive element
			if (increase_osd_value && osd_decode(*pointer, status) == 0x00 ){
				*pointer = *pointer + 1;
				osd_menu_phase = 1; //repaint the screen again
			}
			if (decrease_osd_value && osd_decode(*pointer, status) == 0x01 ){
				*pointer = *pointer - 1;
				osd_menu_phase = 1; //repaint the screen again
			}
			break;
		case 1:	//adjust TEXT or INVERT
			if (increase_osd_value && osd_decode(*pointer, status) == TEXT ){	//increase requested and currently on TEXT
				*pointer = *pointer | (0x02);	//flip the 2nd bit on
				osd_menu_phase = 1; //repaint the screen again
			}
				if (decrease_osd_value && osd_decode(*pointer, status) == INVERT ){	//decrease requested and currently on INVERT
				*pointer = *pointer ^ (0x02) ;	//flip the 2nd bit off
				osd_menu_phase = 1; //repaint the screen again
			}
			break;
		case 2:	//adjust positionX
			if (increase_osd_value && osd_decode(*pointer, status) != 30 ){
				*pointer = (((*pointer >> 2) + 1) << 2) + (*pointer & 0x03);
				osd_menu_phase = 1; //repaint the screen again
			}
			if (decrease_osd_value && osd_decode(*pointer, status) != 0 ){
				*pointer = (((*pointer >> 2) - 1) << 2) + (*pointer & 0x03);
				osd_menu_phase = 1; //repaint the screen again
			}
			break;
		case 3:	//adjust positionY
			if (increase_osd_value && osd_decode(*pointer, status) != 15 ){
				*pointer = (((*pointer >> 7) + 1) << 7) + (*pointer & 0x7F);
				osd_menu_phase = 1; //repaint the screen again
			}
			if (decrease_osd_value && osd_decode(*pointer, status) != 0 ){
				*pointer = (((*pointer >> 7) - 1) << 7) + (*pointer & 0x7F);
				osd_menu_phase = 1; //repaint the screen again
			}
			break;
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
  	if (menu_type == BF_PIDS) fast_fprint(data_buffer, 5, pid_scale(pointer->axis[data_index[index][1]], pid_scale_index[index]), 0);
  	if (menu_type == SW_RATES){
  		if (index < 3) fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]], 0);
  		else fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]] + FLT_EPSILON, 2);
  	}
  	if (menu_type == ROUNDED) fast_fprint(data_buffer, 5, pointer->axis[data_index[index][1]] + FLT_EPSILON, 2);
  	osd_print_data(data_buffer, 5, grid_selection(grid[index][0], grid[index][1]), print_position[index][0], print_position[index][1]);
  	osd_menu_phase++;
}

void print_osd_adjustable_float(uint8_t string_element_qty, uint8_t data_element_qty, float *pointer[], const uint8_t grid[data_element_qty][2], const uint8_t print_position[data_element_qty][2], uint8_t precision){
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
	uint8_t data_buffer[5];
	uint8_t index = osd_menu_phase-string_element_qty-1;
	fast_fprint(data_buffer, 5, *pointer[index] + FLT_EPSILON, precision);
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
  extern float vbattfilt;
  extern float vbattfilt_corr;
  extern float vbatt_comp;
  extern float lipo_cell_count;
  extern int lowbatt;
  extern float throttle;
  extern int binding_while_armed;

  switch (osd_display_phase) //phase starts at 2, RRR gesture subtracts 1 to enter the menu, RRR again or DDD subtracts 1 to clear the screen and return to regular display
  {
  case 0: //osd screen clears, resets to regular display, and resets wizard and menu starting points
    osd_display_reset();
    osd_clear_screen();
    break; //screen has been cleared for this loop - break out of display function

  case 1:                //osd menu is active
    if (flash_feature_1) //setup wizard
    {
    	last_display_phase = 2;
    	print_osd_menu_strings(10, 9, main_menu_labels, main_menu_positions);
    	if(osd_menu_phase == 11)osd_select_menu_item(7,main_menu_map, MAIN_MENU);
    } else {
    	last_display_phase = 2;
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
		  if (decode_active_element(*callsign1)){
			  uint8_t callsign_done = print_osd_callsign();
			  if(callsign_done) osd_display_element++;
		  }else{
			  osd_display_element++;
		  }
		  break; //screen has been displayed for this loop - break out of display function

	  case 1:
		  if (decode_active_element(*fuelgauge_volts)) {
			  uint8_t osd_fuelgauge_volts[5];
			  fast_fprint(osd_fuelgauge_volts, 4, vbatt_comp, 1);
			  osd_fuelgauge_volts[4] = 'V';
			  osd_print_data(osd_fuelgauge_volts, 5, decode_attribute(*fuelgauge_volts), decode_positionx(*fuelgauge_volts) + 3, decode_positiony(*fuelgauge_volts));
		  }
		  osd_display_element++;
		  break;

	  case 2:
		  if (decode_active_element(*fuelgauge_volts)) {
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
		  if (decode_active_element(*filtered_volts)) {
			  uint8_t osd_filtered_volts[5];
			  fast_fprint(osd_filtered_volts, 4, vbattfilt_corr, 1);
			  osd_filtered_volts[4] = 'V';
			  osd_print_data(osd_filtered_volts, 5, decode_attribute(*filtered_volts), decode_positionx(*filtered_volts) + 3, decode_positiony(*filtered_volts));
		  }
		  osd_display_element++;
		  break;

	  case 4:
		  if (decode_active_element(*filtered_volts)) {
			  if (lowbatt != last_lowbatt_state2) {
				  uint8_t osd_cellcount2[2] = {lipo_cell_count + 48, 'S'};
				  if (!lowbatt) {
					  osd_print_data(osd_cellcount2, 2, decode_attribute(*filtered_volts), decode_positionx(*filtered_volts), decode_positiony(*filtered_volts));
				  } else {
					  osd_print_data(osd_cellcount2, 2, BLINK | INVERT, decode_positionx(*filtered_volts), decode_positiony(*filtered_volts));
				  }
				  last_lowbatt_state2 = lowbatt;
			  }
		  }
		  osd_display_element++;
		  break;

	  case 5:
		  if (decode_active_element(*exact_volts)) {
			  uint8_t osd_exact_volts[5];
			  fast_fprint(osd_exact_volts, 4, vbattfilt, 1);
			  osd_exact_volts[4] = 'V';
			  osd_print_data(osd_exact_volts, 5, decode_attribute(*exact_volts), decode_positionx(*exact_volts) + 3, decode_positiony(*exact_volts));
		  }
		  osd_display_element++;
		  break;

	  case 6:
		  if (decode_active_element(*exact_volts)) {
			  if (lowbatt != last_lowbatt_state3) {
				  uint8_t osd_cellcount3[2] = {lipo_cell_count + 48, 'S'};
				  if (!lowbatt) {
					  osd_print_data(osd_cellcount3, 2, decode_attribute(*exact_volts), decode_positionx(*exact_volts), decode_positiony(*exact_volts));
				  } else {
					  osd_print_data(osd_cellcount3, 2, BLINK | INVERT, decode_positionx(*exact_volts), decode_positiony(*exact_volts));
				  }
				  last_lowbatt_state3 = lowbatt;
			  }
		  }
		  osd_display_element++;
		  break;

	  case 7:
		  if (decode_active_element(*stopwatch)) {
			  uint8_t osd_stopwatch[5];
			  fast_fprint(osd_stopwatch, 5, debug.totaltime, 0);
			  osd_stopwatch[4] = 112; //Z+23 is fly hr
			  osd_print_data(osd_stopwatch, 5, decode_attribute(*stopwatch), decode_positionx(*stopwatch), decode_positiony(*stopwatch));
		  }
		  osd_display_element++;
		  break;

	  case 8:
		  if (decode_active_element(*flight_mode)){
			  uint8_t flightmode_done = print_osd_flightmode();
			  if(flightmode_done) osd_display_element++;
		  }else{
			  osd_display_element++;
		  }
		  break;

	  case 9:
		  if (decode_active_element(*osd_throttle)){
			  uint8_t osd_throttle_value[5];
			  fast_fprint(osd_throttle_value, 5, (throttle * 100.0f), 0);
			  osd_throttle_value[4] = 4;
			  osd_print_data(osd_throttle_value, 5, decode_attribute(*osd_throttle), decode_positionx(*osd_throttle), decode_positiony(*osd_throttle));
		  }
		  osd_display_element++;
		  break;

	  case 10:
		  if (decode_active_element(*arm_disarm)){
			  uint8_t system_status_done = print_osd_system_status();
			  if(system_status_done) osd_display_element++;
		  }else{
			  osd_display_element++;
		  }
		  break;

	  case 11:  //end of regular display - display_trigger counter sticks here till it wraps
		  display_trigger++;
		  if (display_trigger == 0) osd_display_element = 1;
		  break;
	  }
    break;
//**********************************************************************************************************************************************************************************************
//																				OSD MENUS BELOW THIS POINT
//**********************************************************************************************************************************************************************************************
  case 3:		//pids profile menu
	  last_display_phase = 1;
	  print_osd_menu_strings(3, 2, pid_profiles_labels, pid_profiles_positions);
	  if (osd_menu_phase == 4)osd_submenu_select (&profile.pid.pid_profile, 2 , pid_submenu_map);
    break;

  case 4:		//pids profiles
	  last_display_phase = 3;
	  if(profile.pid.pid_profile == PID_PROFILE_1) print_osd_menu_strings(8, 4, pid_profile1_labels, pid_profile_positions);
	  else print_osd_menu_strings(8, 4, pid_profile2_labels, pid_profile_positions);
	  print_osd_adjustable_vectors(BF_PIDS, 8, 9, get_pid_term(pid_profile_data_index[osd_menu_phase-9][0]), pid_profile_data_index, pid_profile_grid, pid_profile_data_positions);
	  if (osd_menu_phase == 18) osd_vector_adjust(get_pid_term(osd_cursor), 3, 3, BF_PIDS, pid_profile_adjust_limits);
    break;

  case 5:		//filters menu
	  last_display_phase = 1;
	  print_osd_menu_strings(3, 2, filter_temp_labels, filter_temp_positions);
    break;

  case 6:		//main rates menu
	  last_display_phase = 1;
	  print_osd_menu_strings(3, 2, rates_profile_labels, rates_profile_positions);
	  if (osd_menu_phase == 4)osd_submenu_select (&profile.rate.mode, 2 , rates_submenu_map);
    break;

  case 7:		//silverware rates submenu
	  last_display_phase = 6;
	  print_osd_menu_strings(8, 4, sw_rates_labels, sw_rates_positions);
	  print_osd_adjustable_vectors(SW_RATES, 8, 9, get_sw_rate_term(sw_rates_data_index[osd_menu_phase-9][0]), sw_rates_data_index, sw_rates_grid, sw_rates_data_positions);
	  if (osd_menu_phase == 18) osd_vector_adjust(get_sw_rate_term(osd_cursor), 3, 3, SW_RATES, sw_rates_adjust_limits);
    break;

  case 8:		//betaflight rates submenu
	  last_display_phase = 6;
	  print_osd_menu_strings(8, 4, bf_rates_labels, bf_rates_positions);
	  print_osd_adjustable_vectors(ROUNDED, 8, 9, get_bf_rate_term(bf_rates_data_index[osd_menu_phase-9][0]), bf_rates_data_index, bf_rates_grid, bf_rates_data_positions);
	  if (osd_menu_phase == 18) osd_vector_adjust(get_bf_rate_term(osd_cursor), 3, 3, ROUNDED, bf_rates_adjust_limits);
    break;

  case 9:		//flight modes menu
	  last_display_phase = 1;
	  print_osd_menu_strings(12, 11, flight_modes_labels, flight_modes_positions);
	  print_osd_adjustable_enums (12, 10, get_aux_status(profile.channel.aux[flight_modes_aux_items[osd_menu_phase-13]]), flight_modes_grid, flight_modes_data_positions);
	  if (osd_menu_phase==23)osd_enum_adjust(&profile.channel.aux[flight_modes_aux_items[osd_cursor-1]], 10, flight_modes_aux_limits);
    break;

  case 10:		//osd elements menu
	  last_display_phase = 1;
	  print_osd_menu_strings(5, 4, osd_elements_menu_labels, osd_elements_menu_positions);
	  if (osd_menu_phase == 6) osd_select_menu_item(4,osd_elements_map, SUB_MENU);
    break;

  case 11:		//vtx
	  last_display_phase = 1;
	  print_osd_menu_strings(3, 2, vtx_temp_labels, vtx_temp_positions);
    break;

  case 12:		//special features
	  last_display_phase = 1;
	  print_osd_menu_strings(6, 5, special_features_labels, special_features_positions);
	  if (osd_menu_phase == 7) osd_select_menu_item(5,special_features_map, SUB_MENU);
      break;

  case 13:		//stick boost profiles
	  last_display_phase = 12;
	  print_osd_menu_strings(3, 2, stickboost_labels, stickboost_profile_positions);
	  if (osd_menu_phase == 4) osd_submenu_select (&profile.pid.stick_profile, 2 , stickboost_submenu_map);
      break;

  case 14:		//adjustable stick boost profiles
	  last_display_phase = 13;
	  if(profile.pid.stick_profile == STICK_PROFILE_OFF) print_osd_menu_strings(7, 3, stickboost1_labels, stickboost_positions);
	  else print_osd_menu_strings(7, 3, stickboost2_labels, stickboost_positions);
	  print_osd_adjustable_vectors(ROUNDED, 7, 6, get_stick_profile_term(stickboost_data_index[osd_menu_phase-8][0]), stickboost_data_index, stickboost_grid, stickboost_data_positions);
	  if (osd_menu_phase == 14) osd_vector_adjust(get_stick_profile_term(osd_cursor), 2, 3, ROUNDED, stickboost_adjust_limits);
	  break;

  case 15:		//add or remove osd elements to display
	  last_display_phase = 10;
	  print_osd_menu_strings(12, 11, osd_display_labels, osd_display_positions);
	  print_osd_adjustable_enums (12, 10, get_decode_element_string(osd_element[osd_elements_active_items[osd_menu_phase-13]], ACTIVE), osd_display_grid, osd_display_data_positions);
	  if (osd_menu_phase==23) osd_encoded_adjust(&osd_element[osd_elements_active_items[osd_cursor-1]], 10, 1, ACTIVE);
	  break;

  case 16:		//edit element positions
	  last_display_phase = 10;
	  print_osd_menu_strings(14, 11, osd_position_labels, osd_position_adjust_positions);
	  print_osd_adjustable_enums (14, 20,get_decode_element_string(osd_element[osd_position_active_items[osd_menu_phase-15]], osd_position_index[osd_menu_phase-15]), osd_position_grid, osd_position_data_positions);
	  if (osd_menu_phase==35 && osd_select > 0) osd_encoded_adjust(&osd_element[osd_elements_active_items[osd_cursor-1]], 10, 2, osd_select+1);
	  break;

  case 17:		//edit display text style
	  last_display_phase = 10;
	  print_osd_menu_strings(12, 11, osd_text_style, osd_text_style_positions);
	  print_osd_adjustable_enums (12, 10, get_decode_element_string(osd_element[osd_elements_active_items[osd_menu_phase-13]], ATTRIBUTE), osd_display_grid, osd_display_data_positions);
	  if (osd_menu_phase==23) osd_encoded_adjust(&osd_element[osd_elements_active_items[osd_cursor-1]], 10, 1, ATTRIBUTE);
	  break;

  case 18:		//edit callsign text
	  last_display_phase = 10;
	  print_osd_menu_strings(23, 2, osd_callsign_edit_labels, osd_callsign_edit_positions);
	  print_osd_callsign_adjustable(23, 20, osd_callsign_grid, osd_callsign_edit_data_positions);
	  if ( osd_menu_phase== 44) osd_encoded_adjust_callsign();
	  break;

  case 19:		//edit lowbatt threshold
	  last_display_phase = 12;
	  print_osd_menu_strings(3, 2, lowbatt_labels, lowbatt_positions);
	  print_osd_adjustable_float(3, 1, low_batt_ptr, lowbatt_grid, lowbatt_data_positions, 1);
	  if (osd_menu_phase == 5) osd_float_adjust(low_batt_ptr, 1, 1, lowbatt_adjust_limits, 0.1);
	  break;

  case 20:		//edit levelmode
	  last_display_phase = 12;
	  print_osd_menu_strings(3, 2, level_submenu_labels, level_submenu_positions);
	  if (osd_menu_phase == 4) osd_select_menu_item(2,level_submenu_map, SUB_MENU);
	  break;

  case 21:		//edit torque boost
	  last_display_phase = 12;
	  print_osd_menu_strings(3, 2, torqueboost_labels, torqueboost_positions);
	  print_osd_adjustable_float(3, 1, torqueboost_ptr, torqueboost_grid, torqueboost_data_positions, 1);
	  if (osd_menu_phase == 5) osd_float_adjust(torqueboost_ptr, 1, 1, torqueboost_adjust_limits, 0.1);
	  break;

  case 22:		//edit digital idle
	  last_display_phase = 12;
	  print_osd_menu_strings(3, 2, motoridle_labels, motoridle_positions);
	  print_osd_adjustable_float(3, 1, motoridle_ptr, motoridle_grid, motoridle_data_positions, 1);
	  if (osd_menu_phase == 5) osd_float_adjust(motoridle_ptr, 1, 1, motoridle_adjust_limits, 0.1);
	  break;

  case 23:		//edit level max angle
	  last_display_phase = 20;
	  print_osd_menu_strings(3, 2, maxangle_labels, maxangle_positions);
	  print_osd_adjustable_float(3, 1, level_maxangle_ptr, maxangle_grid, maxangle_data_positions, 0);
	  if (osd_menu_phase == 5) osd_float_adjust(level_maxangle_ptr, 1, 1, maxangle_adjust_limits, 1.0);
	  break;

  case 24:		//edit level strength
	  last_display_phase = 20;
	  print_osd_menu_strings(6, 3, levelmode_labels, levelmode_positions);
	  print_osd_adjustable_float(6, 4, level_pid_ptr, levelmode_grid, levelmode_data_positions, 1);
	  if (osd_menu_phase == 11) osd_float_adjust(level_pid_ptr, 2, 2, levelmode_adjust_limits, 0.5);
	  break;


  }
if (osd_display_phase !=2 && rx_aux_on(AUX_ARMING)) binding_while_armed = 1;	//final safety check to disallow arming during OSD operation
} //end osd_display()
//******************************************************************************************************************************
#endif
