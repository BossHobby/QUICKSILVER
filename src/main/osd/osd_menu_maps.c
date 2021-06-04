#include "osd_menu_maps.h"
#include "filter.h"
#include "profile.h"
#include "stdio.h"
#include "vtx.h"

extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;
//main menu maps
const char main_menu_labels[10][21] = {"MENU", "VTX", "PIDS", "FILTERS", "RATES", "FLIGHT MODES", "OSD ELEMENTS", "SPECIAL FEATURES", "RC LINK", "SAVE AND EXIT"};
const uint8_t main_menu_positions[10][2] = {{13, 1}, {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}, {7, 8}, {7, 9}, {7, 10}, {7, 11}};
const uint8_t main_menu_map[] = {11, 3, 5, 6, 9, 10, 12, 31}; //case numbers for {vtx, pids, filters, rates, flight modes, osd elements, special features}

//pid profiles submenu map
const char pid_profiles_labels[3][21] = {{"PID PROFILES"}, {"PID PROFILE 1"}, {"PID PROFILE 2"}};
const uint8_t pid_profiles_positions[3][2] = {{9, 1}, {7, 4}, {7, 5}};
const uint8_t pid_submenu_map[] = {4, 4}; //describes the menu case to call next for each submenu option

//adjust increments used in vector adjust functions
const float bf_pids_increments[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const float sw_rates_increments[] = {10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
const float rounded_increments[] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

// pids profiles map
const char pid_profile1_labels[8][21] = {{"PID PROFILE 1"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"KP"}, {"KI"}, {"KD"}, {"SAVE AND EXIT"}};
const char pid_profile2_labels[8][21] = {{"PID PROFILE 2"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"KP"}, {"KI"}, {"KD"}, {"SAVE AND EXIT"}};
const uint8_t pid_profile_positions[8][2] = {{9, 1}, {10, 4}, {16, 4}, {23, 4}, {4, 6}, {4, 7}, {4, 8}, {2, 14}};
const uint8_t pid_profile_data_index[9][2] = {{1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2}};
const uint8_t pid_profile_grid[9][2] = {{1, 1}, {2, 1}, {3, 1}, {1, 2}, {2, 2}, {3, 2}, {1, 3}, {2, 3}, {3, 3}};
const uint8_t pid_profile_data_positions[9][2] = {{10, 6}, {16, 6}, {22, 6}, {10, 7}, {16, 7}, {22, 7}, {10, 8}, {16, 8}, {22, 8}};
const float pid_profile_adjust_limits[9][2] = {{0.0, 400.0}, {0.0, 400.0}, {0.0, 400.0}, {0.0, 100.0}, {0.0, 100.0}, {0.0, 100.0}, {0.0, 120.0}, {0.0, 120.0}, {0.0, 120.0}};

//filters submenu map
const char filter_labels[3][21] = {{"FILTERS"}, {"GYRO"}, {"D-TERM"}};
const uint8_t filter_positions[3][2] = {{11, 1}, {7, 4}, {7, 5}};
const uint8_t filter_submenu_map[2] = {28, 29};

//rates submenu map
const char rates_profile_labels[3][21] = {{"RATES"}, {"SILVERWARE"}, {"BETAFLIGHT"}};
const uint8_t rates_profile_positions[3][2] = {{13, 1}, {7, 4}, {7, 5}};
const uint8_t rates_submenu_map[] = {7, 8};

//silverware rates map
const char sw_rates_labels[8][21] = {{"SILVERWARE RATES"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"RATE"}, {"ACRO EXPO"}, {"ANGLE EXPO"}, {"SAVE AND EXIT"}};
const uint8_t sw_rates_positions[8][2] = {{7, 1}, {14, 4}, {19, 4}, {25, 4}, {2, 6}, {2, 7}, {2, 8}, {2, 14}};
const uint8_t sw_rates_data_index[9][2] = {{1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2}};
const uint8_t sw_rates_grid[9][2] = {{1, 1}, {2, 1}, {3, 1}, {1, 2}, {2, 2}, {3, 2}, {1, 3}, {2, 3}, {3, 3}};
const uint8_t sw_rates_data_positions[9][2] = {{14, 6}, {19, 6}, {24, 6}, {13, 7}, {18, 7}, {23, 7}, {13, 8}, {18, 8}, {23, 8}};
const float sw_rates_adjust_limits[9][2] = {{0, 1800.0}, {0, 1800.0}, {0, 1800.0}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99}, {0, 0.99}};

//betaflight rates map
const char bf_rates_labels[8][21] = {{"BETAFLIGHT RATES"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"RC RATE"}, {"SUPER RATE"}, {"EXPO"}, {"SAVE AND EXIT"}};
const uint8_t bf_rates_positions[8][2] = {{7, 1}, {14, 4}, {19, 4}, {25, 4}, {2, 6}, {2, 7}, {2, 8}, {2, 14}};
const uint8_t bf_rates_data_index[9][2] = {{1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 1}, {3, 2}};
const uint8_t bf_rates_grid[9][2] = {{1, 1}, {2, 1}, {3, 1}, {1, 2}, {2, 2}, {3, 2}, {1, 3}, {2, 3}, {3, 3}};
const uint8_t bf_rates_data_positions[9][2] = {{13, 6}, {18, 6}, {23, 6}, {13, 7}, {18, 7}, {23, 7}, {13, 8}, {18, 8}, {23, 8}};
const float bf_rates_adjust_limits[9][2] = {{0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 3.0}, {0, 0.99}, {0, 0.99}, {0, 0.99}};

//flight modes map
uint8_t *flight_modes_ptr[10] = {&profile.receiver.aux[0], &profile.receiver.aux[1], &profile.receiver.aux[2], &profile.receiver.aux[3], &profile.receiver.aux[4], &profile.receiver.aux[5], &profile.receiver.aux[9], &profile.receiver.aux[10], &profile.receiver.aux[11], &profile.receiver.aux[12]};
const char flight_modes_labels[12][21] = {"FLIGHT MODES", "ARMING", "IDLE UP", "LEVELMODE", "RACEMODE", "HORIZON", "STICK BOOST", "BUZZER", "TURTLE", "MOTOR TEST", "RSSI", "SAVE AND EXIT"};
const uint8_t flight_modes_positions[12][2] = {{9, 1}, {4, 2}, {4, 3}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 10}, {4, 11}, {4, 14}};
const uint8_t flight_modes_data_positions[10][2] = {{17, 2}, {17, 3}, {17, 4}, {17, 5}, {17, 6}, {17, 7}, {17, 8}, {17, 9}, {17, 10}, {17, 11}};
const uint8_t flight_modes_aux_limits[] = {11, 14, 14, 14, 14, 14, 14, 14, 14, 12}; //from aux_channel_t
const uint8_t flight_modes_aux_items[] = {0, 1, 2, 3, 4, 5, 9, 10, 11, 12};         //from aux_function_t
const uint8_t flight_modes_grid[10][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10}};

//osd elements submenu map
const char osd_elements_menu_labels[5][21] = {{"OSD ELEMENTS"}, {"ADD OR REMOVE"}, {"EDIT POSITIONS"}, {"EDIT TEXT STYLE"}, {"EDIT CALLSIGN"}};
const uint8_t osd_elements_menu_positions[5][2] = {{9, 1}, {7, 4}, {7, 5}, {7, 6}, {7, 7}};
const uint8_t osd_elements_map[] = {15, 16, 17, 18};

//osd element add/remove & text/invert submenu map
const char osd_display_labels[12][21] = {{"OSD DISPLAY ITEMS"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"GYRO TEMP"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"}};
const uint8_t osd_display_positions[12][2] = {{6, 1}, {4, 2}, {4, 3}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 10}, {4, 11}, {4, 14}};
const uint8_t osd_display_data_positions[10][2] = {{20, 2}, {20, 3}, {20, 4}, {20, 5}, {20, 6}, {20, 7}, {20, 8}, {20, 9}, {20, 10}, {20, 11}};
const uint8_t osd_display_grid[10][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9}, {1, 10}};
const uint8_t osd_elements_active_items[] = {0, 6, 7, 8, 9, 10, 11, 12, 13, 14};

//osd positions submenu map
const char osd_position_labels[14][21] = {{"OSD POSITIONS"}, {"ADJ X"}, {"ADJ Y"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"EXACT VOLTS"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"}};
const uint8_t osd_position_adjust_positions[14][2] = {{1, 1}, {18, 1}, {24, 1}, {3, 2}, {3, 3}, {3, 4}, {3, 5}, {3, 6}, {3, 7}, {3, 8}, {3, 9}, {3, 10}, {3, 11}, {3, 14}};
const uint8_t osd_position_grid[20][2] = {{1, 1}, {2, 1}, {1, 2}, {2, 2}, {1, 3}, {2, 3}, {1, 4}, {2, 4}, {1, 5}, {2, 5}, {1, 6}, {2, 6}, {1, 7}, {2, 7}, {1, 8}, {2, 8}, {1, 9}, {2, 9}, {1, 10}, {2, 10}};
const uint8_t osd_position_data_positions[20][2] = {{20, 2}, {26, 2}, {20, 3}, {26, 3}, {20, 4}, {26, 4}, {20, 5}, {26, 5}, {20, 6}, {26, 6}, {20, 7}, {26, 7}, {20, 8}, {26, 8}, {20, 9}, {26, 9}, {20, 10}, {26, 10}, {20, 11}, {26, 11}};
const uint8_t osd_position_active_items[] = {0, 0, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14};
const uint8_t osd_position_index[20] = {2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3};

//osd text style submenu map
const char osd_text_style[12][21] = {{"OSD TEXT STYLE"}, {"CALLSIGN"}, {"FUELGAUGE VOLTS"}, {"FILTERED VOLTS"}, {"EXACT VOLTS"}, {"FLIGHT MODE"}, {"RSSI"}, {"STOPWATCH"}, {"SYSTEM STATUS"}, {"THROTTLE"}, {"VTX"}, {"SAVE AND EXIT"}};
const uint8_t osd_text_style_positions[12][2] = {{8, 1}, {4, 2}, {4, 3}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 10}, {4, 11}, {4, 14}};

//osd callsign edit submenu map
const char osd_callsign_edit_labels[23][21] = {{"CALLSIGN"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"-"}, {"EDIT:"}, {"SAVE AND EXIT"}};
const uint8_t osd_callsign_edit_positions[23][2] = {{11, 1}, {8, 6}, {9, 6}, {10, 6}, {11, 6}, {12, 6}, {13, 6}, {14, 6}, {15, 6}, {16, 6}, {17, 6}, {18, 6}, {19, 6}, {20, 6}, {21, 6}, {22, 6}, {23, 6}, {24, 6}, {25, 6}, {26, 6}, {27, 6}, {1, 5}, {1, 7}};
const uint8_t osd_callsign_grid[20][2] = {{1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {6, 1}, {7, 1}, {8, 1}, {9, 1}, {10, 1}, {11, 1}, {12, 1}, {13, 1}, {14, 1}, {15, 1}, {16, 1}, {17, 1}, {18, 1}, {19, 1}, {20, 1}};
const uint8_t osd_callsign_edit_data_positions[20][2] = {{8, 5}, {9, 5}, {10, 5}, {11, 5}, {12, 5}, {13, 5}, {14, 5}, {15, 5}, {16, 5}, {17, 5}, {18, 5}, {19, 5}, {20, 5}, {21, 5}, {22, 5}, {23, 5}, {24, 5}, {25, 5}, {26, 5}, {27, 5}};
const uint8_t callsign_shift_index[20][2] = {{1, 0}, {1, 8}, {1, 16}, {1, 24}, {2, 0}, {2, 8}, {2, 16}, {2, 24}, {3, 0}, {3, 8}, {3, 16}, {3, 24}, {4, 0}, {4, 8}, {4, 16}, {4, 24}, {5, 0}, {5, 8}, {5, 16}, {5, 24}};

//vtx not configured map
const char vtx_na_labels[3][21] = {{"VTX CONTROLS"}, {"SMART AUDIO"}, {"NOT CONFIGURED"}};
const uint8_t vtx_na_positions[3][2] = {{9, 1}, {7, 4}, {7, 5}};
//vtx menu map
uint8_t *vtx_ptr[4] = {&vtx_settings_copy.band, &vtx_settings_copy.channel, &vtx_settings_copy.power_level, &vtx_settings_copy.pit_mode};
const char vtx_labels[6][21] = {"VTX CONTROLS", "BAND", "CHANNEL", "POWER LEVEL", "PITMODE", "SAVE AND EXIT"};
const uint8_t vtx_positions[6][2] = {{9, 1}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 14}};
const uint8_t vtx_data_positions[4][2] = {{20, 4}, {20, 5}, {20, 6}, {20, 7}};
const uint8_t vtx_limits[4] = {4, 7, 3, 1};
const uint8_t vtx_grid[4][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}};

//special features menu map
const char special_features_labels[8][21] = {{"SPECIAL FEATURES"}, {"STICK BOOST"}, {"MOTOR BOOST"}, {"PID MODIFIERS"}, {"DIGITAL IDLE"}, {"LOW BATTERY"}, {"LEVEL MODE"}, {"TURTLE THROTTLE"}};
const uint8_t special_features_positions[8][2] = {{7, 1}, {7, 3}, {7, 4}, {7, 5}, {7, 6}, {7, 7}, {7, 8}, {7, 9}};
const uint8_t special_features_map[] = {13, 21, 30, 22, 19, 20, 27}; //case numbers for {stickboost}, etc ...adding more soon

//stick boost submenu map
const char stickboost_labels[3][21] = {{"STICK BOOST PROFILES"}, {"AUX OFF PROFILE 1"}, {"AUX ON  PROFILE 2"}};
const uint8_t stickboost_profile_positions[3][2] = {{5, 1}, {7, 4}, {7, 5}};
const uint8_t stickboost_submenu_map[] = {14, 14};

//stick boost map
const char stickboost1_labels[7][21] = {{"BOOST PROFILE 1"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"ACCELERATOR"}, {"TRANSITION"}, {"SAVE AND EXIT"}};
const char stickboost2_labels[7][21] = {{"BOOST PROFILE 2"}, {"ROLL"}, {"PITCH"}, {"YAW"}, {"ACCELERATOR"}, {"TRANSITION"}, {"SAVE AND EXIT"}};
const uint8_t stickboost_positions[7][2] = {{8, 1}, {14, 4}, {19, 4}, {25, 4}, {2, 6}, {2, 8}, {2, 14}};
const uint8_t stickboost_data_index[6][2] = {{1, 0}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}};
const uint8_t stickboost_grid[6][2] = {{1, 1}, {2, 1}, {3, 1}, {1, 2}, {2, 2}, {3, 2}};
const uint8_t stickboost_data_positions[9][2] = {{13, 6}, {18, 6}, {23, 6}, {13, 8}, {18, 8}, {23, 8}};
const float stickboost_adjust_limits[6][2] = {{0, 3.0}, {0, 3.0}, {0, 3.0}, {-1.0, 1.0}, {-1.0, 1.0}, {-1.0, 1.0}};

//low battery map
float *low_batt_ptr[1] = {&profile.voltage.vbattlow};
const char lowbatt_labels[3][21] = {{"LOW BATTERY"}, {"VOLTS/CELL ALERT"}, {"SAVE AND EXIT"}};
const uint8_t lowbatt_positions[3][2] = {{10, 1}, {4, 5}, {4, 14}};
const uint8_t lowbatt_grid[1][2] = {{1, 1}};
const uint8_t lowbatt_data_positions[1][2] = {{21, 5}};
const float lowbatt_adjust_limits[1][2] = {{0, 4.2}};

//levelmode submenu map
const char level_submenu_labels[3][21] = {{"LEVEL MODE"}, {"MAX ANGLE"}, {"LEVEL STRENGTH"}};
const uint8_t level_submenu_positions[3][2] = {{10, 1}, {7, 4}, {7, 5}};
const uint8_t level_submenu_map[] = {23, 24};

//levelmode maxangle map
float *level_maxangle_ptr[1] = {&profile.rate.level_max_angle};
const char maxangle_labels[3][21] = {{"LEVEL MODE"}, {"MAX ANGLE DEGREES"}, {"SAVE AND EXIT"}};
const uint8_t maxangle_positions[3][2] = {{10, 1}, {1, 5}, {1, 14}};
const uint8_t maxangle_grid[6][2] = {{1, 1}};
const uint8_t maxangle_data_positions[1][2] = {{19, 5}};
const float maxangle_adjust_limits[1][2] = {{0, 85.0}};

//levelmode pid map
float *level_pid_ptr[4] = {&profile.pid.small_angle.kp, &profile.pid.small_angle.kd, &profile.pid.big_angle.kp, &profile.pid.big_angle.kd};
const char levelmode_labels[6][21] = {{"LEVEL MODE"}, {"KP"}, {"KD"}, {"SM ANGLE STRENGTH"}, {"LRG ANGLE STRENGTH"}, {"SAVE AND EXIT"}};
const uint8_t levelmode_positions[6][2] = {{10, 1}, {21, 5}, {26, 5}, {1, 6}, {1, 7}, {1, 14}};
const uint8_t levelmode_grid[4][2] = {{1, 1}, {2, 1}, {1, 2}, {2, 2}};
const uint8_t levelmode_data_positions[4][2] = {{19, 6}, {24, 6}, {19, 7}, {24, 7}};
const float levelmode_adjust_limits[4][2] = {{0, 20.0}, {0, 10.0}, {0, 20.0}, {0, 10.0}};

//motor boost menu map
const char motor_boost_labels[3][21] = {{"MOTOR BOOST TYPES"}, {"TORQUE BOOST"}, {"THROTTLE BOOST"}};
const uint8_t motor_boost_positions[3][2] = {{7, 1}, {7, 4}, {7, 5}};
const uint8_t motor_boost_map[] = {25, 26}; //case numbers for TORQUE BOOST, THROTTLE BOOST

//torque boost map
float *torqueboost_ptr[1] = {&profile.motor.torque_boost};
const char torqueboost_labels[3][21] = {{"TORQUE BOOST"}, {"MOTOR TORQUE BOOST"}, {"SAVE AND EXIT"}};
const uint8_t torqueboost_positions[3][2] = {{9, 1}, {4, 5}, {4, 14}};
const uint8_t torqueboost_grid[1][2] = {{1, 1}};
const uint8_t torqueboost_data_positions[1][2] = {{22, 5}};
const float torqueboost_adjust_limits[1][2] = {{0, 3.0}};

//throttle boost map
float *throttleboost_ptr[1] = {&profile.motor.throttle_boost};
const char throttleboost_labels[3][21] = {{"THROTTLE BOOST"}, {"MOTOR THROTTLE BOOST"}, {"SAVE AND EXIT"}};
const uint8_t throttleboost_positions[3][2] = {{8, 1}, {2, 5}, {2, 14}};
const uint8_t throttleboost_grid[1][2] = {{1, 1}};
const uint8_t throttleboost_data_positions[1][2] = {{22, 5}};
const float throttleboost_adjust_limits[1][2] = {{0, 10.0}};

//digital idle map
float *motoridle_ptr[1] = {&profile.motor.digital_idle};
const char motoridle_labels[3][21] = {{"DIGITAL IDLE"}, {"MOTOR IDLE %"}, {"SAVE AND EXIT"}};
const uint8_t motoridle_positions[3][2] = {{9, 1}, {4, 5}, {4, 14}};
const uint8_t motoridle_grid[1][2] = {{1, 1}};
const uint8_t motoridle_data_positions[1][2] = {{17, 5}};
const float motoridle_adjust_limits[1][2] = {{0, 25.0}};

//turtle map
float *turtlethrottle_ptr[1] = {&profile.motor.turtle_throttle_percent};
const char turtlethrottle_labels[3][21] = {{"TURTLE MODE"}, {"TURTLE THROTTLE %"}, {"SAVE AND EXIT"}};
const uint8_t turtlethrottle_positions[3][2] = {{9, 1}, {4, 5}, {4, 14}};
const uint8_t turtlethrottle_grid[1][2] = {{1, 1}};
const uint8_t turtlethrottle_data_positions[1][2] = {{22, 5}};
const float turtlethrottle_adjust_limits[1][2] = {{0, 100}};

//gyro filter map
float pointer_redirect = POINTER_REDIRECT;
float *gyrofilter_ptr[4] = {&pointer_redirect, &profile.filter.gyro[0].cutoff_freq, &pointer_redirect, &profile.filter.gyro[1].cutoff_freq};
uint8_t *gyrofilter_ptr2[4] = {&profile.filter.gyro[0].type, 0, &profile.filter.gyro[1].type, 0};
const uint8_t gyrofilter_reboot_request[4] = {1, 0, 1, 0};
const char gyrofilter_labels[6][21] = {{"GYRO FILTERS"}, {"PASS 1 TYPE"}, {"PASS 1 FREQ"}, {"PASS 2 TYPE"}, {"PASS 2 FREQ"}, {"SAVE AND EXIT"}};
const char gyrofilter_type_labels[3][21] = {"NONE", " PT1", " PT2"};
const uint8_t gyrofilter_positions[6][2] = {{9, 1}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 14}};
const uint8_t gyrofilter_grid[4][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}};
const uint8_t gyrofilter_data_positions[4][2] = {{18, 4}, {18, 5}, {18, 6}, {18, 7}};
const float gyrofilter_adjust_limits[4][2] = {{0, 2}, {50, 500}, {0, 2}, {50, 500}};

//dterm filter map
float *dtermfilter_ptr[7] = {&pointer_redirect, &profile.filter.dterm[0].cutoff_freq, &pointer_redirect, &profile.filter.dterm[1].cutoff_freq, &pointer_redirect, &profile.filter.dterm_dynamic_min, &profile.filter.dterm_dynamic_max};
uint8_t *dtermfilter_ptr2[7] = {&profile.filter.dterm[0].type, 0, &profile.filter.dterm[1].type, 0, &profile.filter.dterm_dynamic_enable, 0, 0};
const uint8_t dtermfilter_reboot_request[7] = {1, 0, 1, 0, 1, 0, 0};
const char dtermfilter_labels[9][21] = {{"D-TERM FILTERS"}, {"PASS 1 TYPE"}, {"PASS 1 FREQ"}, {"PASS 2 TYPE"}, {"PASS 2 FREQ"}, {"DYNAMIC"}, {"FREQ MIN"}, {"FREQ MAX"}, {"SAVE AND EXIT"}};
const char dtermfilter_type_labels[3][21] = {"NONE", " PT1", " PT2"};
const uint8_t dtermfilter_positions[9][2] = {{8, 1}, {4, 3}, {4, 4}, {4, 5}, {4, 6}, {4, 7}, {4, 8}, {4, 9}, {4, 14}};
const uint8_t dtermfilter_grid[7][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
const uint8_t dtermfilter_data_positions[7][2] = {{18, 3}, {18, 4}, {18, 5}, {18, 6}, {18, 7}, {18, 8}, {18, 9}};
const float dtermfilter_adjust_limits[7][2] = {{0, 2}, {50, 500}, {0, 2}, {50, 500}, {0, 1}, {50, 500}, {50, 500}};

//pid modifiers map
float *pidmodify_ptr[4] = {&pointer_redirect, &pointer_redirect, &profile.pid.throttle_dterm_attenuation.tda_breakpoint, &profile.pid.throttle_dterm_attenuation.tda_percent};
uint8_t *pidmodify_ptr2[4] = {&profile.voltage.pid_voltage_compensation, &profile.pid.throttle_dterm_attenuation.tda_active, 0, 0};
const uint8_t pidmodify_reboot_request[4] = {0, 0, 0, 0};
const char pidmodify_labels[6][21] = {{"PID MODIFIERS"}, {"P-TERM VOLTAGE COMP"}, {"THROTTLE D ATTENUATE"}, {"TDA BREAKPOINT"}, {"TDA PERCENT"}, {"SAVE AND EXIT"}};
const char pidmodify_type_labels[2][21] = {" NONE ", "ACTIVE"};
const uint8_t pidmodify_positions[6][2] = {{8, 1}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 14}};
const uint8_t pidmodify_grid[4][2] = {{1, 1}, {1, 2}, {1, 3}, {1, 4}};
const uint8_t pidmodify_data_positions[4][2] = {{23, 4}, {23, 5}, {23, 6}, {23, 7}};
const float pidmodify_adjust_limits[4][2] = {{0, 1}, {0, 1}, {0, 1}, {0, 1}};

//rc link map
const char rc_link_labels[3][21] = {{"RC LINK"}, {"STICK CALIBRATION"}, {"RSSI SOURCE"}};
const uint8_t rc_link_positions[3][2] = {{12, 1}, {7, 4}, {7, 5}};
const uint8_t rc_link_map[] = {33, 32};

//rssi source map
uint8_t *rssi_source_ptr[2] = {&profile.receiver.lqi_source, &profile.receiver.aux[12]};
const char rssi_menu_labels[4][21] = {{"RSSI SOURCE"}, {"RSSI FROM"}, {"SELECT AUX"}, {"SAVE AND EXIT"}};
const uint8_t rssi_menu_positions[4][2] = {{10, 1}, {4, 4}, {4, 5}, {4, 14}};
const uint8_t rssi_source_limits[] = {2, 12};
const uint8_t rssi_source_data_positions[2][2] = {{16, 4}, {16, 5}};
const uint8_t rssi_source_data_grid[2][2] = {{1, 1}, {1, 2}};

//stick wizard map 1 - enter/exit
const char stick_wizard_labels_1[3][21] = {{"STICK CALIBRATION"}, {"LEFT TO EXIT"}, {"RIGHT TO BEGIN"}};
const uint8_t stick_wizard_positions_1[3][2] = {{7, 1}, {9, 5}, {8, 7}};

//stick wizard map 2 - move sticks to calibrate
const char stick_wizard_labels_2[3][21] = {{"RECORDING"}, {"MOVE STICKS"}, {"TO EXTENTS"}};
const uint8_t stick_wizard_positions_2[3][2] = {{10, 3}, {9, 5}, {9, 7}};

//stick wizard map 3 - move sticks to test
const char stick_wizard_labels_3[3][21] = {{"TESTING CALIBRATION"}, {"MOVE STICKS"}, {"TO ALL 4 CORNERS"}};
const uint8_t stick_wizard_positions_3[3][2] = {{6, 3}, {9, 5}, {7, 7}};

//stick wizard map 4 - calibration success
const char stick_wizard_labels_4[3][21] = {{"STICK CALIBRATION"}, {"CALIBRATION"}, {"SUCCESS"}};
const uint8_t stick_wizard_positions_4[3][2] = {{7, 1}, {10, 5}, {12, 7}};

//stick wizard map 5 - calibration failed
const char stick_wizard_labels_5[3][21] = {{"STICK CALIBRATION"}, {"CALIBRATION"}, {"FAILED"}};
const uint8_t stick_wizard_positions_5[3][2] = {{7, 1}, {10, 5}, {12, 7}};

