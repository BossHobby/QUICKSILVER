#include "osd_menu_maps.h"
#include "filter.h"
#include "profile.h"
#include "stdio.h"
#include "vtx.h"

#define MENU_SIZE(menu) const uint32_t menu##_size = (sizeof(menu) / sizeof(osd_label_t));

extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

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