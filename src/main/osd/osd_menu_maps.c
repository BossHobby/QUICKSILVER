#include "osd_menu_maps.h"
#include "filter.h"
#include "profile.h"
#include "stdio.h"
#include "vtx.h"

#define MENU_SIZE(menu) const uint32_t menu##_size = (sizeof(menu) / sizeof(osd_label_t));

extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

// osd element add/remove & text/invert submenu map
const osd_label_t osd_display_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "OSD DISPLAY ITEMS",
        .pos = {6, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "CALLSIGN",
        .pos = {4, 2},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FUELGAUGE VOLTS",
        .pos = {4, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FILTERED VOLTS",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "GYRO TEMP",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FLIGHT MODE",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RSSI",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STOPWATCH",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SYSTEM STATUS",
        .pos = {4, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "THROTTLE",
        .pos = {4, 10},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "VTX",
        .pos = {4, 11},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(osd_display_labels);

const uint8_t osd_display_data_positions[10][2] = {
    {20, 2},
    {20, 3},
    {20, 4},
    {20, 5},
    {20, 6},
    {20, 7},
    {20, 8},
    {20, 9},
    {20, 10},
    {20, 11},
};
const uint8_t osd_display_grid[10][2] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
    {1, 5},
    {1, 6},
    {1, 7},
    {1, 8},
    {1, 9},
    {1, 10},
};
const uint8_t osd_elements_active_items[] = {0, 6, 7, 8, 9, 10, 11, 12, 13, 14};

// osd positions submenu map
const osd_label_t osd_position_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "OSD POSITIONS",
        .pos = {1, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ADJ X",
        .pos = {18, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ADJ Y",
        .pos = {24, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "CALLSIGN",
        .pos = {3, 2},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FUELGAUGE VOLTS",
        .pos = {3, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FILTERED VOLTS",
        .pos = {3, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EXACT VOLTS",
        .pos = {3, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FLIGHT MODE",
        .pos = {3, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RSSI",
        .pos = {3, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STOPWATCH",
        .pos = {3, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SYSTEM STATUS",
        .pos = {3, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "THROTTLE",
        .pos = {3, 10},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "VTX",
        .pos = {3, 11},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {3, 14},
    },
};
MENU_SIZE(osd_position_labels);

const uint8_t osd_position_grid[20][2] = {
    {1, 1},
    {2, 1},
    {1, 2},
    {2, 2},
    {1, 3},
    {2, 3},
    {1, 4},
    {2, 4},
    {1, 5},
    {2, 5},
    {1, 6},
    {2, 6},
    {1, 7},
    {2, 7},
    {1, 8},
    {2, 8},
    {1, 9},
    {2, 9},
    {1, 10},
    {2, 10},
};
const uint8_t osd_position_data_positions[20][2] = {
    {20, 2},
    {26, 2},
    {20, 3},
    {26, 3},
    {20, 4},
    {26, 4},
    {20, 5},
    {26, 5},
    {20, 6},
    {26, 6},
    {20, 7},
    {26, 7},
    {20, 8},
    {26, 8},
    {20, 9},
    {26, 9},
    {20, 10},
    {26, 10},
    {20, 11},
    {26, 11},
};
const uint8_t osd_position_active_items[] = {0, 0, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14};
const uint8_t osd_position_index[20] = {2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3, 2, 3};

// osd text style submenu map
const osd_label_t osd_text_style[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "OSD TEXT STYLE",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "CALLSIGN",
        .pos = {4, 2},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FUELGAUGE VOLTS",
        .pos = {4, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FILTERED VOLTS",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EXACT VOLTS",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FLIGHT MODE",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RSSI",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STOPWATCH",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SYSTEM STATUS",
        .pos = {4, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "THROTTLE",
        .pos = {4, 10},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "VTX",
        .pos = {4, 11},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(osd_text_style);

// osd callsign edit submenu map
const osd_label_t osd_callsign_edit_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "CALLSIGN",
        .pos = {11, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {8, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {9, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {10, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {11, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {12, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {13, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {14, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {15, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {16, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {17, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {18, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {19, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {20, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {21, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {22, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {23, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {24, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {25, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {26, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "-",
        .pos = {27, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EDIT:",
        .pos = {1, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {1, 7},
    },
};
MENU_SIZE(osd_callsign_edit_labels);

const uint8_t osd_callsign_grid[20][2] = {
    {1, 1},
    {2, 1},
    {3, 1},
    {4, 1},
    {5, 1},
    {6, 1},
    {7, 1},
    {8, 1},
    {9, 1},
    {10, 1},
    {11, 1},
    {12, 1},
    {13, 1},
    {14, 1},
    {15, 1},
    {16, 1},
    {17, 1},
    {18, 1},
    {19, 1},
    {20, 1},
};
const uint8_t osd_callsign_edit_data_positions[20][2] = {
    {8, 5},
    {9, 5},
    {10, 5},
    {11, 5},
    {12, 5},
    {13, 5},
    {14, 5},
    {15, 5},
    {16, 5},
    {17, 5},
    {18, 5},
    {19, 5},
    {20, 5},
    {21, 5},
    {22, 5},
    {23, 5},
    {24, 5},
    {25, 5},
    {26, 5},
    {27, 5},
};

// low battery map
float *low_batt_ptr[1] = {&profile.voltage.vbattlow};
const osd_label_t lowbatt_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "LOW BATTERY",
        .pos = {10, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "VOLTS/CELL ALERT",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(lowbatt_labels);

const uint8_t lowbatt_grid[1][2] = {
    {1, 1},
};
const uint8_t lowbatt_data_positions[1][2] = {
    {21, 5},
};
const float lowbatt_adjust_limits[1][2] = {
    {0, 4.2},
};

// levelmode maxangle map
float *level_maxangle_ptr[1] = {&profile.rate.level_max_angle};
const osd_label_t maxangle_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "LEVEL MODE",
        .pos = {10, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MAX ANGLE DEGREES",
        .pos = {1, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {1, 14},
    },
};
MENU_SIZE(maxangle_labels);

const uint8_t maxangle_grid[6][2] = {
    {1, 1},
};
const uint8_t maxangle_data_positions[1][2] = {
    {19, 5},
};
const float maxangle_adjust_limits[1][2] = {
    {0, 85.0},
};

// levelmode pid map
float *level_pid_ptr[4] = {
    &profile.pid.small_angle.kp,
    &profile.pid.small_angle.kd,
    &profile.pid.big_angle.kp,
    &profile.pid.big_angle.kd,
};
const osd_label_t levelmode_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "LEVEL MODE",
        .pos = {10, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "KP",
        .pos = {21, 5},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "KD",
        .pos = {26, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SM ANGLE STRENGTH",
        .pos = {1, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "LRG ANGLE STRENGTH",
        .pos = {1, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {1, 14},
    },
};
MENU_SIZE(levelmode_labels);

const uint8_t levelmode_grid[4][2] = {
    {1, 1},
    {2, 1},
    {1, 2},
    {2, 2},
};
const uint8_t levelmode_data_positions[4][2] = {
    {19, 6},
    {24, 6},
    {19, 7},
    {24, 7},
};
const float levelmode_adjust_limits[4][2] = {
    {0, 20.0},
    {0, 10.0},
    {0, 20.0},
    {0, 10.0},
};

// torque boost map
float *torqueboost_ptr[1] = {&profile.motor.torque_boost};
const osd_label_t torqueboost_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "TORQUE BOOST",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MOTOR TORQUE BOOST",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(torqueboost_labels);

const uint8_t torqueboost_grid[1][2] = {
    {1, 1},
};
const uint8_t torqueboost_data_positions[1][2] = {
    {22, 5},
};
const float torqueboost_adjust_limits[1][2] = {
    {0, 3.0},
};

// throttle boost map
float *throttleboost_ptr[1] = {&profile.motor.throttle_boost};
const osd_label_t throttleboost_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "THROTTLE BOOST",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MOTOR THROTTLE BOOST",
        .pos = {2, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(throttleboost_labels);

const uint8_t throttleboost_grid[1][2] = {
    {1, 1},
};
const uint8_t throttleboost_data_positions[1][2] = {
    {22, 5},
};
const float throttleboost_adjust_limits[1][2] = {
    {0, 10.0},
};

// digital idle map
float *motoridle_ptr[1] = {&profile.motor.digital_idle};
const osd_label_t motoridle_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "DIGITAL IDLE",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MOTOR IDLE %",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(motoridle_labels);

const uint8_t motoridle_grid[1][2] = {
    {1, 1},
};
const uint8_t motoridle_data_positions[1][2] = {
    {17, 5},
};
const float motoridle_adjust_limits[1][2] = {
    {0, 25.0},
};

// turtle map
float *turtlethrottle_ptr[1] = {&profile.motor.turtle_throttle_percent};
const osd_label_t turtlethrottle_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "TURTLE MODE",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TURTLE THROTTLE %",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(turtlethrottle_labels);

const uint8_t turtlethrottle_grid[1][2] = {
    {1, 1},
};
const uint8_t turtlethrottle_data_positions[1][2] = {
    {22, 5},
};
const float turtlethrottle_adjust_limits[1][2] = {
    {0, 100},
};

// rssi source map
uint8_t *rssi_source_ptr[2] = {
    &profile.receiver.lqi_source,
    &profile.receiver.aux[12],
};
const osd_label_t rssi_menu_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "RSSI SOURCE",
        .pos = {10, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RSSI FROM",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SELECT AUX",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(rssi_menu_labels);

const uint8_t rssi_source_limits[] = {2, 12};
const uint8_t rssi_source_data_positions[2][2] = {
    {16, 4},
    {16, 5},
};
const uint8_t rssi_source_data_grid[2][2] = {
    {1, 1},
    {1, 2},
};

// stick wizard map 1 - enter/exit
const osd_label_t stick_wizard_labels_1[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "STICK CALIBRATION",
        .pos = {7, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "LEFT TO EXIT",
        .pos = {9, 5},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "RIGHT TO BEGIN",
        .pos = {8, 7},
    },
};
MENU_SIZE(stick_wizard_labels_1);

// stick wizard map 2 - move sticks to calibrate
const osd_label_t stick_wizard_labels_2[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "RECORDING",
        .pos = {10, 3},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "MOVE STICKS",
        .pos = {9, 5},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "TO EXTENTS",
        .pos = {9, 7},
    },
};
MENU_SIZE(stick_wizard_labels_2);

// stick wizard map 3 - move sticks to test
const osd_label_t stick_wizard_labels_3[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "TESTING CALIBRATION",
        .pos = {5, 3},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "MOVE STICKS AGAIN",
        .pos = {6, 5},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "TO EXTENTS",
        .pos = {9, 7},
    },
};
MENU_SIZE(stick_wizard_labels_3);

// stick wizard map 4 - calibration success
const osd_label_t stick_wizard_labels_4[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "STICK CALIBRATION",
        .pos = {7, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "CALIBRATION",
        .pos = {10, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "SUCCESS",
        .pos = {12, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PUSH LEFT TO EXIT",
        .pos = {7, 8},
    },
};
MENU_SIZE(stick_wizard_labels_4);

// stick wizard map 5 - calibration failed
const osd_label_t stick_wizard_labels_5[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "STICK CALIBRATION",
        .pos = {7, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "CALIBRATION",
        .pos = {10, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "FAILED",
        .pos = {12, 6},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PUSH LEFT TO EXIT",
        .pos = {7, 8},
    },
};
MENU_SIZE(stick_wizard_labels_5);