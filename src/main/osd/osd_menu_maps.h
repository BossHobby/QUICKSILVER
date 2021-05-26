#include "stdio.h"

//main menu maps
extern const char main_menu_labels[10][21];
extern const uint8_t main_menu_positions[10][2];
extern const uint8_t main_menu_map[];

//pid profiles submenu map
extern const char pid_profiles_labels[3][21];
extern const uint8_t pid_profiles_positions[3][2];
extern const uint8_t pid_submenu_map[];

//adjust increments used in vector adjust functions
extern const float bf_pids_increments[];
extern const float sw_rates_increments[];
extern const float rounded_increments[];

// pids profiles map
extern const char pid_profile1_labels[8][21];
extern const char pid_profile2_labels[8][21];
extern const uint8_t pid_profile_positions[8][2];
extern const uint8_t pid_profile_data_index[9][2];
extern const uint8_t pid_profile_grid[9][2];
extern const uint8_t pid_profile_data_positions[9][2];
extern const float pid_profile_adjust_limits[9][2];

//filters submenu map
extern const char filter_labels[3][21];
extern const uint8_t filter_positions[3][2];
extern const uint8_t filter_submenu_map[2];

//rates submenu map
extern const char rates_profile_labels[3][21];
extern const uint8_t rates_profile_positions[3][2];
extern const uint8_t rates_submenu_map[];

//silverware rates map
extern const char sw_rates_labels[8][21];
extern const uint8_t sw_rates_positions[8][2];
extern const uint8_t sw_rates_data_index[9][2];
extern const uint8_t sw_rates_grid[9][2];
extern const uint8_t sw_rates_data_positions[9][2];
extern const float sw_rates_adjust_limits[9][2];

//betaflight rates map
extern const char bf_rates_labels[8][21];
extern const uint8_t bf_rates_positions[8][2];
extern const uint8_t bf_rates_data_index[9][2];
extern const uint8_t bf_rates_grid[9][2];
extern const uint8_t bf_rates_data_positions[9][2];
extern const float bf_rates_adjust_limits[9][2];

//flight modes map
extern uint8_t *flight_modes_ptr[10];
extern const char flight_modes_labels[12][21];
extern const uint8_t flight_modes_positions[12][2];
extern const uint8_t flight_modes_data_positions[10][2];
extern const uint8_t flight_modes_aux_limits[];
extern const uint8_t flight_modes_aux_items[];
extern const uint8_t flight_modes_grid[10][2];

//osd elements submenu map
extern const char osd_elements_menu_labels[5][21];
extern const uint8_t osd_elements_menu_positions[5][2];
extern const uint8_t osd_elements_map[];

//osd element add/remove & text/invert submenu map
extern const char osd_display_labels[12][21];
extern const uint8_t osd_display_positions[12][2];
extern const uint8_t osd_display_data_positions[10][2];
extern const uint8_t osd_display_grid[10][2];
extern const uint8_t osd_elements_active_items[];

//osd positions submenu map
extern const char osd_position_labels[14][21];
extern const uint8_t osd_position_adjust_positions[14][2];
extern const uint8_t osd_position_grid[20][2];
extern const uint8_t osd_position_data_positions[20][2];
extern const uint8_t osd_position_active_items[];
extern const uint8_t osd_position_index[20];

//osd text style submenu map
extern const char osd_text_style[12][21];
extern const uint8_t osd_text_style_positions[12][2];

//osd callsign edit submenu map
extern const char osd_callsign_edit_labels[23][21];
extern const uint8_t osd_callsign_edit_positions[23][2];
extern const uint8_t osd_callsign_grid[20][2];
extern const uint8_t osd_callsign_edit_data_positions[20][2];
extern const uint8_t callsign_shift_index[20][2];

//vtx submenu map
extern const char vtx_na_labels[3][21];
extern const uint8_t vtx_na_positions[3][2];
//vtx menu map
extern uint8_t *vtx_ptr[4];
extern const char vtx_labels[6][21];
extern const uint8_t vtx_positions[6][2];
extern const uint8_t vtx_data_positions[4][2];
extern const uint8_t vtx_limits[4];
extern const uint8_t vtx_grid[4][2];

//special features menu map
extern const char special_features_labels[8][21];
extern const uint8_t special_features_positions[8][2];
extern const uint8_t special_features_map[];

//stick boost submenu map
extern const char stickboost_labels[3][21];
extern const uint8_t stickboost_profile_positions[3][2];
extern const uint8_t stickboost_submenu_map[];

//stick boost map
extern const char stickboost1_labels[7][21];
extern const char stickboost2_labels[7][21];
extern const uint8_t stickboost_positions[7][2];
extern const uint8_t stickboost_data_index[6][2];
extern const uint8_t stickboost_grid[6][2];
extern const uint8_t stickboost_data_positions[9][2];
extern const float stickboost_adjust_limits[6][2];

//low battery map
extern float *low_batt_ptr[1];
extern const char lowbatt_labels[3][21];
extern const uint8_t lowbatt_positions[3][2];
extern const uint8_t lowbatt_grid[1][2];
extern const uint8_t lowbatt_data_positions[1][2];
extern const float lowbatt_adjust_limits[1][2];

//levelmode submenu map
extern const char level_submenu_labels[3][21];
extern const uint8_t level_submenu_positions[3][2];
extern const uint8_t level_submenu_map[];

//levelmode maxangle map
extern float *level_maxangle_ptr[1];
extern const char maxangle_labels[3][21];
extern const uint8_t maxangle_positions[3][2];
extern const uint8_t maxangle_grid[6][2];
extern const uint8_t maxangle_data_positions[1][2];
extern const float maxangle_adjust_limits[1][2];

//levelmode pid map
extern float *level_pid_ptr[4];
extern const char levelmode_labels[6][21];
extern const uint8_t levelmode_positions[6][2];
extern const uint8_t levelmode_grid[4][2];
extern const uint8_t levelmode_data_positions[4][2];
extern const float levelmode_adjust_limits[4][2];

//motor boost menu map
extern const char motor_boost_labels[3][21];
extern const uint8_t motor_boost_positions[3][2];
extern const uint8_t motor_boost_map[];

//torque boost map
extern float *torqueboost_ptr[1];
extern const char torqueboost_labels[3][21];
extern const uint8_t torqueboost_positions[3][2];
extern const uint8_t torqueboost_grid[1][2];
extern const uint8_t torqueboost_data_positions[1][2];
extern const float torqueboost_adjust_limits[1][2];

//throttle boost map
extern float *throttleboost_ptr[1];
extern const char throttleboost_labels[3][21];
extern const uint8_t throttleboost_positions[3][2];
extern const uint8_t throttleboost_grid[1][2];
extern const uint8_t throttleboost_data_positions[1][2];
extern const float throttleboost_adjust_limits[1][2];

//digital idle map
extern float *motoridle_ptr[1];
extern const char motoridle_labels[3][21];
extern const uint8_t motoridle_positions[3][2];
extern const uint8_t motoridle_grid[1][2];
extern const uint8_t motoridle_data_positions[1][2];
extern const float motoridle_adjust_limits[1][2];

//turtle map
extern float *turtlethrottle_ptr[1];
extern const char turtlethrottle_labels[3][21];
extern const uint8_t turtlethrottle_positions[3][2];
extern const uint8_t turtlethrottle_grid[1][2];
extern const uint8_t turtlethrottle_data_positions[1][2];
extern const float turtlethrottle_adjust_limits[1][2];

//gyro filter map
#define POINTER_REDIRECT -999.0f
//static float pointer_redirect;
extern float *gyrofilter_ptr[4];
extern uint8_t *gyrofilter_ptr2[4];
extern const uint8_t gyrofilter_reboot_request[4];
extern const char gyrofilter_labels[6][21];
extern const char gyrofilter_type_labels[3][21];
extern const uint8_t gyrofilter_positions[6][2];
extern const uint8_t gyrofilter_grid[4][2];
extern const uint8_t gyrofilter_data_positions[4][2];
extern const float gyrofilter_adjust_limits[4][2];

//dterm filter map
extern float *dtermfilter_ptr[7];
extern uint8_t *dtermfilter_ptr2[7];
extern const uint8_t dtermfilter_reboot_request[7];
extern const char dtermfilter_labels[9][21];
extern const char dtermfilter_type_labels[3][21];
extern const uint8_t dtermfilter_positions[9][2];
extern const uint8_t dtermfilter_grid[7][2];
extern const uint8_t dtermfilter_data_positions[7][2];
extern const float dtermfilter_adjust_limits[7][2];

//pid modifiers map
extern float *pidmodify_ptr[4];
extern uint8_t *pidmodify_ptr2[4];
extern const uint8_t pidmodify_reboot_request[4];
extern const char pidmodify_labels[6][21];
extern const char pidmodify_type_labels[2][21];
extern const uint8_t pidmodify_positions[6][2];
extern const uint8_t pidmodify_grid[4][2];
extern const uint8_t pidmodify_data_positions[4][2];
extern const float pidmodify_adjust_limits[4][2];

//rc link map
extern const char rc_link_labels[3][21];
extern const uint8_t rc_link_positions[3][2];
extern const uint8_t rc_link_map[];

//rssi source map
extern uint8_t *rssi_source_ptr[1];
extern const char rssi_menu_labels[3][21];
extern const uint8_t rssi_menu_positions[3][2];
extern const char rssi_source_labels[3][21];
extern const uint8_t rssi_source_limits[];
extern const uint8_t rssi_source_data_positions[1][2];
extern const uint8_t rssi_source_data_grid[1][2];

//stick wizard map
extern const char stick_wizard_labels[1][21];
extern const uint8_t stick_wizard_positions[1][2];
