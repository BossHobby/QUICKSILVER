#include "drv_max7456.h"
#include "drv_time.h"
#include "project.h"
#include "stdio.h"
#include "string.h"
#include "profile.h"
#include "float.h"
#include "util.h"
#include "rx.h"

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
// case state variables for switch logic

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

uint16_t rp_kp_scale(float pid_rate){		//increment by 0.0015923566878981f
	uint16_t scaled_pid_value = round_num(pid_rate * 628.0f);
	return scaled_pid_value;
}

uint16_t yaw_kp_scale(float pid_rate){		//increment by 0.0031847133757962f
	uint16_t scaled_pid_value = round_num(pid_rate * 314.0f);
	return scaled_pid_value;
}

uint16_t ki_scale(float pid_rate){			//increment by 0.02f
	uint16_t scaled_pid_value = round_num(pid_rate * 50.0f);
	return scaled_pid_value;
}

uint16_t kd_scale(float pid_rate){			//increment by 0.0083333333333333f
	uint16_t scaled_pid_value = round_num(pid_rate * 120.0f);
	return scaled_pid_value;
}

float increment_rounded_float(float input){
    float value = (int)(input * 100.0f + 0.5f);
    return (float)(value+1) / 100.0f;
}

float decrement_rounded_float(float input){
    float value = (int)(input * 100.0f + 0.5f);
    return (float)(value-1) / 100.0f;
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
//******************************************************************************************************************************


//******************************************************************************************************************************
// osd submenu pointer logic

void osd_select_menu_item(void) {
  if (osd_select == 1 && flash_feature_1 == 1) //main mmenu
  {
    osd_select = 0; //reset the trigger

    switch (osd_cursor) {
    case 0:	//nothing happens here - case 0 is not used
      break;

    case 1:		//vtx
    	osd_cursor = 0;
    	osd_display_phase = 11;
    	osd_menu_phase = 0;
      break;

    case 2:		//pids
    	osd_cursor = 0;
    	osd_display_phase = 3;
    	osd_menu_phase = 0;
      break;

    case 3:		//filters
    	osd_cursor = 0;
    	osd_display_phase = 5;
    	osd_menu_phase = 0;
      break;

    case 4:		//rates
    	osd_cursor = 0;
    	osd_display_phase = 6;
    	osd_menu_phase = 0;
      break;

    case 5:		//flight modes
    	osd_cursor = 0;
    	osd_display_phase = 9;
    	osd_menu_phase = 0;
      break;

    case 6:		//osd elements
    	osd_cursor = 0;
    	osd_display_phase = 10;
    	osd_menu_phase = 0;
      break;

    case 7:		//special features
    	osd_cursor = 0;
    	osd_display_phase = 12;
    	osd_menu_phase = 0;
      break;

    case 8:		//setup wizard
      flash_feature_1 = 0;
      break;

    case 9:		//save & exit
      osd_save_exit();
      break;
    }
  }
}
//******************************************************************************************************************************


//******************************************************************************************************************************
//osd pids logic

void osd_select_pidprofile_item(void)
{
  if (osd_select == 1) //stick was pushed right to select a rate type
  {
	osd_select = 0;	//reset the trigger
	switch(osd_cursor){
	case 1:
		osd_cursor = 0;	//reset the cursor
		profile.pid.pid_profile = PID_PROFILE_1;	//update profile
		osd_display_phase = 4;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	case 2:
		osd_cursor = 0;
		profile.pid.pid_profile = PID_PROFILE_2;	//update profile
		osd_display_phase = 4;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	}
  }
}


void osd_adjust_pidprofile_item(void)
{
	if(osd_select > 3) {
		osd_select = 3;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	switch(osd_cursor){
	case 1: //adjust row 1 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.roll = profile.pid.pid_rates[profile.pid.pid_profile].kp.roll + 0.0015924f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.roll = profile.pid.pid_rates[profile.pid.pid_profile].kp.roll - 0.0015924f;
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.pitch = profile.pid.pid_rates[profile.pid.pid_profile].kp.pitch + 0.0015924f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.pitch = profile.pid.pid_rates[profile.pid.pid_profile].kp.pitch - 0.0015924f;
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.yaw = profile.pid.pid_rates[profile.pid.pid_profile].kp.yaw + 0.0031847f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kp.yaw = profile.pid.pid_rates[profile.pid.pid_profile].kp.yaw - 0.0031847f;
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 2:	//adjust row 2 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.roll = profile.pid.pid_rates[profile.pid.pid_profile].ki.roll + 0.02f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.roll = profile.pid.pid_rates[profile.pid.pid_profile].ki.roll - 0.02f;
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.pitch = profile.pid.pid_rates[profile.pid.pid_profile].ki.pitch + 0.02f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.pitch = profile.pid.pid_rates[profile.pid.pid_profile].ki.pitch - 0.02f;
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.yaw = profile.pid.pid_rates[profile.pid.pid_profile].ki.yaw + 0.02f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].ki.yaw = profile.pid.pid_rates[profile.pid.pid_profile].ki.yaw - 0.02f;
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 3:	//adjust row 3 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.roll = profile.pid.pid_rates[profile.pid.pid_profile].kd.roll + 0.0083333f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.roll = profile.pid.pid_rates[profile.pid.pid_profile].kd.roll - 0.0083333f;
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.pitch = profile.pid.pid_rates[profile.pid.pid_profile].kd.pitch + 0.0083333f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.pitch = profile.pid.pid_rates[profile.pid.pid_profile].kd.pitch - 0.0083333f;
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.yaw = profile.pid.pid_rates[profile.pid.pid_profile].kd.yaw + 0.0083333f;
			if (decrease_osd_value) profile.pid.pid_rates[profile.pid.pid_profile].kd.yaw = profile.pid.pid_rates[profile.pid.pid_profile].kd.yaw - 0.0083333f;
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 4: //save&exit silverware rates
		if (osd_select == 1){
		osd_save_exit();
		}
		break;
	}

}
//******************************************************************************************************************************


//******************************************************************************************************************************
//osd rates logic

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
	case 1: //adjust row 1 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.silverware.max_rate.roll = profile.rate.silverware.max_rate.roll + 10.0f;
			if (decrease_osd_value) profile.rate.silverware.max_rate.roll = profile.rate.silverware.max_rate.roll - 10.0f;
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.silverware.max_rate.pitch = profile.rate.silverware.max_rate.pitch + 10.0f;
			if (decrease_osd_value) profile.rate.silverware.max_rate.pitch = profile.rate.silverware.max_rate.pitch - 10.0f;
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.silverware.max_rate.yaw = profile.rate.silverware.max_rate.yaw + 10.0f;
			if (decrease_osd_value) profile.rate.silverware.max_rate.yaw = profile.rate.silverware.max_rate.yaw - 10.0f;
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 2:	//adjust row 2 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.silverware.acro_expo.roll = increment_rounded_float(profile.rate.silverware.acro_expo.roll);
			if (decrease_osd_value) profile.rate.silverware.acro_expo.roll = decrement_rounded_float(profile.rate.silverware.acro_expo.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.silverware.acro_expo.pitch = increment_rounded_float(profile.rate.silverware.acro_expo.pitch);
			if (decrease_osd_value) profile.rate.silverware.acro_expo.pitch = decrement_rounded_float(profile.rate.silverware.acro_expo.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.silverware.acro_expo.yaw = increment_rounded_float(profile.rate.silverware.acro_expo.yaw);
			if (decrease_osd_value) profile.rate.silverware.acro_expo.yaw = decrement_rounded_float(profile.rate.silverware.acro_expo.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 3:	//adjust row 3 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.silverware.angle_expo.roll = increment_rounded_float(profile.rate.silverware.angle_expo.roll);
			if (decrease_osd_value) profile.rate.silverware.angle_expo.roll = decrement_rounded_float(profile.rate.silverware.angle_expo.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.silverware.angle_expo.pitch = increment_rounded_float(profile.rate.silverware.angle_expo.pitch);
			if (decrease_osd_value) profile.rate.silverware.angle_expo.pitch = decrement_rounded_float(profile.rate.silverware.angle_expo.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.silverware.angle_expo.yaw = increment_rounded_float(profile.rate.silverware.angle_expo.yaw);
			if (decrease_osd_value) profile.rate.silverware.angle_expo.yaw = decrement_rounded_float(profile.rate.silverware.angle_expo.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 4: //save&exit silverware rates
		if (osd_select == 1){
		osd_save_exit();
		}
		break;
	}
}


void osd_adjust_betaflightrates_item(void)
{
	if(osd_select > 3) {
		osd_select = 3;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	switch(osd_cursor){
	case 1: //adjust row 1 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.betaflight.rc_rate.roll = increment_rounded_float(profile.rate.betaflight.rc_rate.roll);
			if (decrease_osd_value) profile.rate.betaflight.rc_rate.roll = decrement_rounded_float(profile.rate.betaflight.rc_rate.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.betaflight.rc_rate.pitch = increment_rounded_float(profile.rate.betaflight.rc_rate.pitch);
			if (decrease_osd_value) profile.rate.betaflight.rc_rate.pitch = decrement_rounded_float(profile.rate.betaflight.rc_rate.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.betaflight.rc_rate.yaw = increment_rounded_float(profile.rate.betaflight.rc_rate.yaw);
			if (decrease_osd_value) profile.rate.betaflight.rc_rate.yaw = decrement_rounded_float(profile.rate.betaflight.rc_rate.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 2:	//adjust row 2 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.betaflight.super_rate.roll = increment_rounded_float(profile.rate.betaflight.super_rate.roll);
			if (decrease_osd_value) profile.rate.betaflight.super_rate.roll = decrement_rounded_float(profile.rate.betaflight.super_rate.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.betaflight.super_rate.pitch = increment_rounded_float(profile.rate.betaflight.super_rate.pitch);
			if (decrease_osd_value) profile.rate.betaflight.super_rate.pitch = decrement_rounded_float(profile.rate.betaflight.super_rate.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.betaflight.super_rate.yaw = increment_rounded_float(profile.rate.betaflight.super_rate.yaw);
			if (decrease_osd_value) profile.rate.betaflight.super_rate.yaw = decrement_rounded_float(profile.rate.betaflight.super_rate.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 3:	//adjust row 3 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.rate.betaflight.expo.roll = increment_rounded_float(profile.rate.betaflight.expo.roll);
			if (decrease_osd_value) profile.rate.betaflight.expo.roll = decrement_rounded_float(profile.rate.betaflight.expo.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.rate.betaflight.expo.pitch = increment_rounded_float(profile.rate.betaflight.expo.pitch);
			if (decrease_osd_value) profile.rate.betaflight.expo.pitch = decrement_rounded_float(profile.rate.betaflight.expo.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.rate.betaflight.expo.yaw = increment_rounded_float(profile.rate.betaflight.expo.yaw);
			if (decrease_osd_value) profile.rate.betaflight.expo.yaw = decrement_rounded_float(profile.rate.betaflight.expo.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 4: //save&exit silverware rates
		if (osd_select == 1){
		osd_save_exit();
		}
		break;
	}

}
//******************************************************************************************************************************


//******************************************************************************************************************************
//osd special features logic

void osd_select_flightmode(void)
{
	if(osd_select > 1) {
		osd_select = 1;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}

	switch(osd_cursor){
	case 1: //adjust row 1 item based on osd_select value and up/down osd gestures
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_ARMING];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_ARMING] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_12)  {
			i++;
			profile.channel.aux[AUX_ARMING] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 2:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_IDLE_UP];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_IDLE_UP] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_IDLE_UP] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 3:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_LEVELMODE];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_LEVELMODE] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_LEVELMODE] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 4:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_RACEMODE];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_RACEMODE] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_RACEMODE] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 5:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_HORIZON];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_HORIZON] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_HORIZON] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 6:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_PIDPROFILE];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_PIDPROFILE] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_PIDPROFILE] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 7:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_RATES];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_RATES] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_RATES] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 8:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_BUZZER_ENABLE];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_BUZZER_ENABLE] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_BUZZER_ENABLE] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 9:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_STARTFLIP];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_STARTFLIP] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_STARTFLIP] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 10:
		if (osd_select == 1){
		int i = profile.channel.aux[AUX_MOTORS_TO_THROTTLE_MODE];
		if (increase_osd_value && i != AUX_CHANNEL_0)  {
			i--;
			profile.channel.aux[AUX_MOTORS_TO_THROTTLE_MODE] = i;
			}
		if (decrease_osd_value && i != AUX_CHANNEL_OFF)  {
			i++;
			profile.channel.aux[AUX_MOTORS_TO_THROTTLE_MODE] = i;
			}
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 11: //save & exit FLIGHT MODES
		if (osd_select == 1){
		osd_save_exit();
		}
		break;
	}


}
//******************************************************************************************************************************


//******************************************************************************************************************************
//osd special features logic

void osd_select_special_features(void)
{
  if (osd_select == 1) //stick was pushed right to select a rate type
  {
	osd_select = 0;	//reset the trigger
	switch(osd_cursor){
    case 1:		//stick boost profiles
    	osd_cursor = 0;
    	osd_display_phase = 13;
    	osd_menu_phase = 0;
      break;
	}
  }
}
//******************************************************************************************************************************


//******************************************************************************************************************************
//osd stick boost logic

void osd_select_stickboost(void)
{
  if (osd_select == 1) //stick was pushed right to select a rate type
  {
	osd_select = 0;	//reset the trigger
	switch(osd_cursor){
	case 1:
		osd_cursor = 0;	//reset the cursor
		profile.pid.stick_profile = STICK_PROFILE_1;	//update profile
		osd_display_phase = 14;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	case 2:
		osd_cursor = 0;
		profile.pid.stick_profile = STICK_PROFILE_2;	//update profile
		osd_display_phase = 14;	//update display phase to the next menu screen
		osd_menu_phase = 0;	//clear the screen
		break;
	}
  }
}


void osd_adjust_stickprofile_item(void)
{
	if(osd_select > 3) {
		osd_select = 3;	//limit osd select variable from accumulating past 3 columns of adjustable items
		osd_menu_phase = 1; //repaint the screen again
	}
	switch(osd_cursor){
	case 1: //adjust row 1 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 2:	//adjust row 2 items based on osd_select value and up/down osd gestures
		if(osd_select == 1){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.roll = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.roll);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.roll = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.roll);
		}
		if(osd_select == 2){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch);
		}
		if(osd_select == 3){
			if (increase_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw = increment_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw);
			if (decrease_osd_value) profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw = decrement_rounded_float(profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw);
		}
		increase_osd_value = 0;
		decrease_osd_value = 0;
		osd_menu_phase = 1; //repaint the screen again
		break;
	case 3: //save&exit silverware rates
		if (osd_select == 1){
		osd_save_exit();
		}
		break;
	}
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
  extern int disable_arming;
  if(osd_display_phase != 2){
	  disable_arming = 1;			//prevent any unexpected accidental surprises while tinkering in menus
  }else{
	  disable_arming = 0;
  }
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

  case 3:		//pids profile menu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("PID PROFILES", INVERT, 9, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("PID PROFILE 1", user_selection(1, 2), 7, 4);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("PID PROFILE 2", user_selection(2, 2), 7, 5);
          osd_menu_phase++;
          break;
      case 4:
    	  osd_select_pidprofile_item();
    	  break;
      }
    break;

  case 4:		//pids profiles
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
          osd_menu_phase++;
          break;
      case 1:
    	  if(profile.pid.pid_profile == PID_PROFILE_1){
    	  osd_print("PID PROFILE 1", INVERT, 9, 1);
    	  osd_menu_phase++;
    	  }else{
          osd_print("PID PROFILE 2", INVERT, 9, 1);
          osd_menu_phase++;
    	  }
    	  break;
      case 2:
          osd_print("ROLL", TEXT, 10, 4);
          osd_menu_phase++;
          break;
      case 3:
    	  osd_print("PITCH", TEXT, 16, 4);
          osd_menu_phase++;
          break;
      case 4:
    	  osd_print("YAW", TEXT, 23, 4);
          osd_menu_phase++;
          break;
      case 5:
          osd_print("KP", user_selection(1, 4), 4, 6);
          osd_menu_phase++;
          break;
      case 6:
    	  osd_print("KI", user_selection(2, 4), 4, 7);
          osd_menu_phase++;
          break;
      case 7:
          osd_print("KD", user_selection(3, 4), 4, 8);
          osd_menu_phase++;
          break;
      case 8:
          osd_print("SAVE AND EXIT", user_selection(4, 4), 2, 14);
          osd_menu_phase++;
          break;
      case 9:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_roll_kp[4];
          fast_fprint(osd_roll_kp, 5, rp_kp_scale(profile.pid.pid_rates[profile.pid.pid_profile].kp.roll), 0);
          osd_print_data(osd_roll_kp, 4, adjust_selection(1, 1), 10, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_pitch_kp[4];
          fast_fprint(osd_pitch_kp, 5, rp_kp_scale(profile.pid.pid_rates[profile.pid.pid_profile].kp.pitch), 0);
          osd_print_data(osd_pitch_kp, 4, adjust_selection(2, 1), 16, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_yaw_kp[4];
          fast_fprint(osd_yaw_kp, 5, yaw_kp_scale(profile.pid.pid_rates[profile.pid.pid_profile].kp.yaw), 0);
          osd_print_data(osd_yaw_kp, 4, adjust_selection(3, 1), 22, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_roll_ki[4];
          fast_fprint(osd_roll_ki, 5, ki_scale(profile.pid.pid_rates[profile.pid.pid_profile].ki.roll), 0);
          osd_print_data(osd_roll_ki, 4, adjust_selection(1, 2), 10, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_pitch_ki[4];
          fast_fprint(osd_pitch_ki, 5, ki_scale(profile.pid.pid_rates[profile.pid.pid_profile].ki.pitch), 0);
          osd_print_data(osd_pitch_ki, 4, adjust_selection(2, 2), 16, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_yaw_ki[4];
          fast_fprint(osd_yaw_ki, 5, ki_scale(profile.pid.pid_rates[profile.pid.pid_profile].ki.yaw), 0);
          osd_print_data(osd_yaw_ki, 4, adjust_selection(3, 2), 22, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_roll_kd[4];
          fast_fprint(osd_roll_kd, 5, kd_scale(profile.pid.pid_rates[profile.pid.pid_profile].kd.roll), 0);
          osd_print_data(osd_roll_kd, 4, adjust_selection(1, 3), 10, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_pitch_kd[4];
          fast_fprint(osd_pitch_kd, 5, kd_scale(profile.pid.pid_rates[profile.pid.pid_profile].kd.pitch), 0);
          osd_print_data(osd_pitch_kd, 4, adjust_selection(2, 3), 16, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.pid.pid_profile == PID_PROFILE_1 || profile.pid.pid_profile == PID_PROFILE_2){
          uint8_t osd_yaw_kd[4];
          fast_fprint(osd_yaw_kd, 5, kd_scale(profile.pid.pid_rates[profile.pid.pid_profile].kd.yaw), 0);
          osd_print_data(osd_yaw_kd, 4, adjust_selection(3, 3), 22, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_adjust_pidprofile_item();
    	  break;
      }
    break;

  case 5:		//filters menu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          osd_print_data(osd_roll_rate, 4, adjust_selection(1, 1), 14, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_pitch_rate[4];
          fast_fprint(osd_pitch_rate, 5, profile.rate.silverware.max_rate.pitch, 0);
          osd_print_data(osd_pitch_rate, 4, adjust_selection(2, 1), 19, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_yaw_rate[4];
          fast_fprint(osd_yaw_rate, 5, profile.rate.silverware.max_rate.yaw, 0);
          osd_print_data(osd_yaw_rate, 4, adjust_selection(3, 1), 24, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_roll[4];
          fast_fprint(osd_acro_expo_roll, 4, profile.rate.silverware.acro_expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_roll, 4, adjust_selection(1, 2), 14, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_pitch[4];
          fast_fprint(osd_acro_expo_pitch, 4, profile.rate.silverware.acro_expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_pitch, 4, adjust_selection(2, 2), 19, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_acro_expo_yaw[4];
          fast_fprint(osd_acro_expo_yaw, 4, profile.rate.silverware.acro_expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_yaw, 4, adjust_selection(3, 2), 24, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_roll[4];
          fast_fprint(osd_angle_expo_roll, 4, profile.rate.silverware.angle_expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_roll, 4, adjust_selection(1, 3), 14, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_pitch[4];
          fast_fprint(osd_angle_expo_pitch, 4, profile.rate.silverware.angle_expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_pitch, 4, adjust_selection(2, 3), 19, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.rate.mode == RATE_MODE_SILVERWARE){
          uint8_t osd_angle_expo_yaw[4];
          fast_fprint(osd_angle_expo_yaw, 4, profile.rate.silverware.angle_expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_yaw, 4, adjust_selection(3, 3), 24, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_adjust_silverwarerates_item();
    	  break;
      }
    break;

  case 8:		//betaflight rates submenu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          uint8_t osd_roll_rate[4];
          fast_fprint(osd_roll_rate, 4, profile.rate.betaflight.rc_rate.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_rate, 4, adjust_selection(1, 1), 14, 6);
    	  }
          osd_menu_phase++;
          break;
      case 10:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_pitch_rate[4];
          fast_fprint(osd_pitch_rate, 4, profile.rate.betaflight.rc_rate.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_rate, 4, adjust_selection(2, 1), 19, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_yaw_rate[4];
          fast_fprint(osd_yaw_rate, 4, profile.rate.betaflight.rc_rate.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_rate, 4, adjust_selection(3, 1), 24, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_roll[4];
          fast_fprint(osd_acro_expo_roll, 4, profile.rate.betaflight.super_rate.roll + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_roll, 4, adjust_selection(1, 2), 14, 7);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_pitch[4];
          fast_fprint(osd_acro_expo_pitch, 4, profile.rate.betaflight.super_rate.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_pitch, 4, adjust_selection(2, 2), 19, 7);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_acro_expo_yaw[4];
          fast_fprint(osd_acro_expo_yaw, 4, profile.rate.betaflight.super_rate.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_acro_expo_yaw, 4, adjust_selection(3, 2), 24, 7);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_roll[4];
          fast_fprint(osd_angle_expo_roll, 4, profile.rate.betaflight.expo.roll + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_roll, 4, adjust_selection(1, 3), 14, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_pitch[4];
          fast_fprint(osd_angle_expo_pitch, 4, profile.rate.betaflight.expo.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_pitch, 4, adjust_selection(2, 3), 19, 8);
    	  }
          osd_menu_phase++;
          break;
      case 17:
    	  if (profile.rate.mode == RATE_MODE_BETAFLIGHT){
          uint8_t osd_angle_expo_yaw[4];
          fast_fprint(osd_angle_expo_yaw, 4, profile.rate.betaflight.expo.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_angle_expo_yaw, 4, adjust_selection(3, 3), 24, 8);
    	  }
          osd_menu_phase++;
          break;
      case 18:
    	  osd_adjust_betaflightrates_item();
    	  break;
      }
    break;

  case 9:		//flight modes menu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
          osd_menu_phase++;
          break;
      case 1:
    	  osd_print("FLIGHT MODES", INVERT, 9, 1);
    	  osd_menu_phase++;
    	  break;
      case 2:
          osd_print("ARMING", user_selection(1, 11), 4, 2);
          osd_menu_phase++;
          break;
      case 3:
          osd_print("IDLE UP", user_selection(2, 11), 4, 3);
          osd_menu_phase++;
          break;
      case 4:
          osd_print("LEVELMODE", user_selection(3, 11), 4, 4);
          osd_menu_phase++;
          break;
      case 5:
          osd_print("RACEMODE", user_selection(4, 11), 4, 5);
          osd_menu_phase++;
          break;
      case 6:
          osd_print("HORIZON", user_selection(5, 11), 4, 6);
          osd_menu_phase++;
          break;
      case 7:
          osd_print("STICK BOOST", user_selection(6, 11), 4, 7);
          osd_menu_phase++;
          break;
      case 8:
          osd_print("HIGH RATES", user_selection(7, 11), 4, 8);
          osd_menu_phase++;
          break;
      case 9:
          osd_print("BUZZER", user_selection(8, 11), 4, 9);
          osd_menu_phase++;
          break;
      case 10:
          osd_print("TURTLE", user_selection(9, 11), 4, 10);
          osd_menu_phase++;
          break;
      case 11:
          osd_print("MOTOR TEST", user_selection(10, 11), 4, 11);
          osd_menu_phase++;
          break;
      case 12:
          osd_print("SAVE AND EXIT", user_selection(11, 11), 4, 14);
          osd_menu_phase++;
          break;
      case 13:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_ARMING]), adjust_selection(1, 1), 17, 2);
          osd_menu_phase++;
          break;
      case 14:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_IDLE_UP]), adjust_selection(1, 2), 17, 3);
          osd_menu_phase++;
          break;
      case 15:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_LEVELMODE]), adjust_selection(1, 3), 17, 4);
          osd_menu_phase++;
          break;
      case 16:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_RACEMODE]), adjust_selection(1, 4), 17, 5);
          osd_menu_phase++;
          break;
      case 17:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_HORIZON]), adjust_selection(1, 5), 17, 6);
          osd_menu_phase++;
          break;
      case 18:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_PIDPROFILE]), adjust_selection(1, 6), 17, 7);
          osd_menu_phase++;
          break;
      case 19:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_RATES]), adjust_selection(1, 7), 17, 8);
          osd_menu_phase++;
          break;
      case 20:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_BUZZER_ENABLE]), adjust_selection(1, 8), 17, 9);
          osd_menu_phase++;
          break;
      case 21:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_STARTFLIP]), adjust_selection(1, 9), 17, 10);
          osd_menu_phase++;
          break;
      case 22:
    	  osd_print(get_aux_status(profile.channel.aux[AUX_MOTORS_TO_THROTTLE_MODE]), adjust_selection(1, 10), 17, 11);
          osd_menu_phase++;
          break;
      case 23:
    	  osd_select_flightmode();
    	  break;
      }
    break;

  case 10:		//osd elements menu
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
    	  osd_select_special_features();
    	  break;
      }
      break;

  case 13:		//stick accelerator profiles
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
    	  osd_select_stickboost();
    	  break;
      }
      break;

  case 14:		//stick boost profiles
      switch (osd_menu_phase) {
      case 0:
          osd_clear();
          extern unsigned long lastlooptime;
          lastlooptime = gettime();
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
          uint8_t osd_roll_accel[4];
          fast_fprint(osd_roll_accel, 4, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_accel, 4, adjust_selection(1, 1), 14, 6);
    	  }
          osd_menu_phase++;
          break;
      case 11:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_pitch_accel[4];
          fast_fprint(osd_pitch_accel, 4, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_accel, 4, adjust_selection(2, 1), 19, 6);
    	  }
          osd_menu_phase++;
          break;
      case 12:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_yaw_accel[4];
          fast_fprint(osd_yaw_accel, 4, profile.pid.stick_rates[profile.pid.stick_profile].accelerator.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_accel, 4, adjust_selection(3, 1), 24, 6);
    	  }
          osd_menu_phase++;
          break;
      case 13:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_roll_trans[4];
          fast_fprint(osd_roll_trans, 4, profile.pid.stick_rates[profile.pid.stick_profile].transition.roll + FLT_EPSILON, 2);
          osd_print_data(osd_roll_trans, 4, adjust_selection(1, 2), 14, 8);
    	  }
          osd_menu_phase++;
          break;
      case 14:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_pitch_trans[4];
          fast_fprint(osd_pitch_trans, 4, profile.pid.stick_rates[profile.pid.stick_profile].transition.pitch + FLT_EPSILON, 2);
          osd_print_data(osd_pitch_trans, 4, adjust_selection(2, 2), 19, 8);
    	  }
          osd_menu_phase++;
          break;
      case 15:
    	  if (profile.pid.stick_profile == STICK_PROFILE_1 || profile.pid.stick_profile == STICK_PROFILE_2){
          uint8_t osd_yaw_trans[4];
          fast_fprint(osd_yaw_trans, 4, profile.pid.stick_rates[profile.pid.stick_profile].transition.yaw + FLT_EPSILON, 2);
          osd_print_data(osd_yaw_trans, 4, adjust_selection(3, 2), 24, 8);
    	  }
          osd_menu_phase++;
          break;
      case 16:
    	  osd_adjust_stickprofile_item();
    	  break;
      }
    break;

  }

} //end osd_display()
//******************************************************************************************************************************
#endif
