#pragma once

#include <stdio.h>

typedef enum {
  OSD_SCREEN_CLEAR,
  OSD_SCREEN_MAIN_MENU,
  OSD_SCREEN_REGULAR,

  OSD_SCREEN_PID_PROFILE,
  OSD_SCREEN_PID,
  OSD_SCREEN_FILTERS,
  OSD_SCREEN_RATES,
  OSD_SCREEN_SW_RATES,
  OSD_SCREEN_BF_RATES,
  OSD_SCREEN_FLIGHT_MODES,
  OSD_SCREEN_ELEMENTS,
  OSD_SCREEN_VTX,
  OSD_SCREEN_SPECIAL_FEATURES,
  OSD_SCREEN_STICK_BOOST,
  OSD_SCREEN_STICK_BOOST_ADJUST,
  OSD_SCREEN_ELEMENTS_ADD_REMOVE,
  OSD_SCREEN_ELEMENTS_POSITION,
  OSD_SCREEN_ELEMENTS_STYLE,
  OSD_SCREEN_CALLSIGN,
  OSD_SCREEN_LOWBAT,
  OSD_SCREEN_LEVEL_MODE,
  OSD_SCREEN_MOTOR_BOOST,
  OSD_SCREEN_DIGITAL_IDLE,
  OSD_SCREEN_LEVEL_MAX_ANGLE,
  OSD_SCREEN_LEVEL_STRENGTH,
  OSD_SCREEN_TORQUE_BOOST,
  OSD_SCREEN_THROTTLE_BOOST,
  OSD_SCREEN_TURTLE_THROTTLE,
  OSD_SCREEN_GYRO_FILTER,
  OSD_SCREEN_DTERM_FILTER,
  OSD_SCREEN_PID_MODIFIER,
  OSD_SCREEN_RC_LINK,
  OSD_SCREEN_RSSI,
  OSD_SCREEN_STICK_WIZARD,
  OSD_SCREEN_STICK_WIZARD_CALIBRATION,
  OSD_SCREEN_STICK_CONFIRM,
  OSD_SCREEN_STICK_RESULT
} osd_screens_t;

typedef enum {
  OSD_LABEL_HEADER,
  OSD_LABEL_ACTIVE,
  OSD_LABEL_INACTIVE,
} osd_label_type_t;

typedef struct {
  osd_label_type_t type;
  const char *text;
  uint8_t pos[2];
} osd_label_t;

// main menu maps
extern const osd_label_t main_menu_labels[];
extern const uint32_t main_menu_labels_size;
extern const uint8_t main_menu_map[];

// pid profiles submenu map
extern const osd_label_t pid_profiles_labels[];
extern const uint32_t pid_profiles_labels_size;
extern const uint8_t pid_submenu_map[];

// adjust increments used in vector adjust functions
extern const float bf_pids_increments[];
extern const float sw_rates_increments[];
extern const float rounded_increments[];

// pids profiles map
extern const osd_label_t pid_profile1_labels[];
extern const uint32_t pid_profile1_labels_size;
extern const osd_label_t pid_profile2_labels[];
extern const uint32_t pid_profile2_labels_size;
extern const uint8_t pid_profile_data_index[9][2];
extern const uint8_t pid_profile_grid[9][2];
extern const uint8_t pid_profile_data_positions[9][2];
extern const float pid_profile_adjust_limits[9][2];

// filters submenu map
extern const osd_label_t filter_labels[];
extern const uint32_t filter_labels_size;
extern const uint8_t filter_submenu_map[2];

// rates submenu map
extern const osd_label_t rates_profile_labels[];
extern const uint32_t rates_profile_labels_size;
extern const uint8_t rates_submenu_map[];

// silverware rates map
extern const osd_label_t sw_rates_labels[];
extern const uint32_t sw_rates_labels_size;
extern const uint8_t sw_rates_data_index[9][2];
extern const uint8_t sw_rates_grid[9][2];
extern const uint8_t sw_rates_data_positions[9][2];
extern const float sw_rates_adjust_limits[9][2];

// betaflight rates map
extern const osd_label_t bf_rates_labels[];
extern const uint32_t bf_rates_labels_size;
extern const uint8_t bf_rates_data_index[9][2];
extern const uint8_t bf_rates_grid[9][2];
extern const uint8_t bf_rates_data_positions[9][2];
extern const float bf_rates_adjust_limits[9][2];

// flight modes map
extern uint8_t *flight_modes_ptr[10];
extern const osd_label_t flight_modes_labels[];
extern const uint32_t flight_modes_labels_size;
extern const uint8_t flight_modes_data_positions[10][2];
extern const uint8_t flight_modes_aux_limits[];
extern const uint8_t flight_modes_aux_items[];
extern const uint8_t flight_modes_grid[10][2];

// osd elements submenu map
extern const osd_label_t osd_elements_menu_labels[];
extern const uint32_t osd_elements_menu_labels_size;
extern const uint8_t osd_elements_map[];

// osd element add/remove & text/invert submenu map
extern const osd_label_t osd_display_labels[];
extern const uint32_t osd_display_labels_size;
extern const uint8_t osd_display_data_positions[10][2];
extern const uint8_t osd_display_grid[10][2];
extern const uint8_t osd_elements_active_items[];

// osd positions submenu map
extern const osd_label_t osd_position_labels[];
extern const uint32_t osd_position_labels_size;
extern const uint8_t osd_position_grid[20][2];
extern const uint8_t osd_position_data_positions[20][2];
extern const uint8_t osd_position_active_items[];
extern const uint8_t osd_position_index[20];

// osd text style submenu map
extern const osd_label_t osd_text_style[];
extern const uint32_t osd_text_style_size;

// osd callsign edit submenu map
extern const osd_label_t osd_callsign_edit_labels[];
extern const uint32_t osd_callsign_edit_labels_size;
extern const uint8_t osd_callsign_grid[20][2];
extern const uint8_t osd_callsign_edit_data_positions[20][2];
extern const uint8_t callsign_shift_index[20][2];

// vtx submenu map
extern const osd_label_t vtx_na_labels[];
extern const uint32_t vtx_na_labels_size;

// vtx menu map
extern uint8_t *vtx_ptr[4];
extern const osd_label_t vtx_labels[];
extern const uint32_t vtx_labels_size;
extern const uint8_t vtx_data_positions[4][2];
extern const uint8_t vtx_limits[4];
extern const uint8_t vtx_grid[4][2];

// special features menu map
extern const osd_label_t special_features_labels[];
extern const uint32_t special_features_labels_size;
extern const uint8_t special_features_map[];

// stick boost submenu map
extern const osd_label_t stickboost_labels[];
extern const uint32_t stickboost_labels_size;
extern const uint8_t stickboost_submenu_map[];

// stick boost map
extern const osd_label_t stickboost1_labels[];
extern const uint32_t stickboost1_labels_size;
extern const osd_label_t stickboost2_labels[];
extern const uint32_t stickboost2_labels_size;
extern const uint8_t stickboost_data_index[6][2];
extern const uint8_t stickboost_grid[6][2];
extern const uint8_t stickboost_data_positions[9][2];
extern const float stickboost_adjust_limits[6][2];

// low battery map
extern float *low_batt_ptr[1];
extern const osd_label_t lowbatt_labels[];
extern const uint32_t lowbatt_labels_size;
extern const uint8_t lowbatt_grid[1][2];
extern const uint8_t lowbatt_data_positions[1][2];
extern const float lowbatt_adjust_limits[1][2];

// levelmode submenu map
extern const osd_label_t level_submenu_labels[];
extern const uint32_t level_submenu_labels_size;
extern const uint8_t level_submenu_map[];

// levelmode maxangle map
extern float *level_maxangle_ptr[1];
extern const osd_label_t maxangle_labels[];
extern const uint32_t maxangle_labels_size;
extern const uint8_t maxangle_grid[6][2];
extern const uint8_t maxangle_data_positions[1][2];
extern const float maxangle_adjust_limits[1][2];

// levelmode pid map
extern float *level_pid_ptr[4];
extern const osd_label_t levelmode_labels[];
extern const uint32_t levelmode_labels_size;
extern const uint8_t levelmode_grid[4][2];
extern const uint8_t levelmode_data_positions[4][2];
extern const float levelmode_adjust_limits[4][2];

// motor boost menu map
extern const osd_label_t motor_boost_labels[];
extern const uint32_t motor_boost_labels_size;
extern const uint8_t motor_boost_map[];

// torque boost map
extern float *torqueboost_ptr[1];
extern const osd_label_t torqueboost_labels[];
extern const uint32_t torqueboost_labels_size;
extern const uint8_t torqueboost_grid[1][2];
extern const uint8_t torqueboost_data_positions[1][2];
extern const float torqueboost_adjust_limits[1][2];

// throttle boost map
extern float *throttleboost_ptr[1];
extern const osd_label_t throttleboost_labels[];
extern const uint32_t throttleboost_labels_size;
extern const uint8_t throttleboost_grid[1][2];
extern const uint8_t throttleboost_data_positions[1][2];
extern const float throttleboost_adjust_limits[1][2];

// digital idle map
extern float *motoridle_ptr[1];
extern const osd_label_t motoridle_labels[];
extern const uint32_t motoridle_labels_size;
extern const uint8_t motoridle_grid[1][2];
extern const uint8_t motoridle_data_positions[1][2];
extern const float motoridle_adjust_limits[1][2];

// turtle map
extern float *turtlethrottle_ptr[1];
extern const osd_label_t turtlethrottle_labels[];
extern const uint32_t turtlethrottle_labels_size;
extern const uint8_t turtlethrottle_grid[1][2];
extern const uint8_t turtlethrottle_data_positions[1][2];
extern const float turtlethrottle_adjust_limits[1][2];

// gyro filter map
#define POINTER_REDIRECT -999.0f
// static float pointer_redirect;
extern float *gyrofilter_ptr[4];
extern uint8_t *gyrofilter_ptr2[4];
extern const uint8_t gyrofilter_reboot_request[4];
extern const osd_label_t gyrofilter_labels[];
extern const uint32_t gyrofilter_labels_size;
extern const char gyrofilter_type_labels[3][21];
extern const uint8_t gyrofilter_grid[4][2];
extern const uint8_t gyrofilter_data_positions[4][2];
extern const float gyrofilter_adjust_limits[4][2];

// dterm filter map
extern float *dtermfilter_ptr[7];
extern uint8_t *dtermfilter_ptr2[7];
extern const uint8_t dtermfilter_reboot_request[7];
extern const osd_label_t dtermfilter_labels[];
extern const uint32_t dtermfilter_labels_size;
extern const char dtermfilter_type_labels[3][21];
extern const uint8_t dtermfilter_grid[7][2];
extern const uint8_t dtermfilter_data_positions[7][2];
extern const float dtermfilter_adjust_limits[7][2];

// pid modifiers map
extern float *pidmodify_ptr[4];
extern uint8_t *pidmodify_ptr2[4];
extern const uint8_t pidmodify_reboot_request[4];
extern const osd_label_t pidmodify_labels[];
extern const uint32_t pidmodify_labels_size;
extern const char pidmodify_type_labels[2][21];
extern const uint8_t pidmodify_grid[4][2];
extern const uint8_t pidmodify_data_positions[4][2];
extern const float pidmodify_adjust_limits[4][2];

// rc link map
extern const osd_label_t rc_link_labels[];
extern const uint32_t rc_link_labels_size;
extern const uint8_t rc_link_positions[3][2];
extern const uint8_t rc_link_map[];

// rssi source map
extern uint8_t *rssi_source_ptr[2];
extern const osd_label_t rssi_menu_labels[];
extern const uint32_t rssi_menu_labels_size;
extern const uint8_t rssi_source_limits[];
extern const uint8_t rssi_source_data_positions[2][2];
extern const uint8_t rssi_source_data_grid[2][2];

// stick wizard map 1
extern const osd_label_t stick_wizard_labels_1[];
extern const uint32_t stick_wizard_labels_1_size;

// stick wizard map 2
extern const osd_label_t stick_wizard_labels_2[];
extern const uint32_t stick_wizard_labels_2_size;

// stick wizard map 3
extern const osd_label_t stick_wizard_labels_3[];
extern const uint32_t stick_wizard_labels_3_size;

// stick wizard map 4
extern const osd_label_t stick_wizard_labels_4[];
extern const uint32_t stick_wizard_labels_4_size;

// stick wizard map 5
extern const osd_label_t stick_wizard_labels_5[];
extern const uint32_t stick_wizard_labels_5_size;
