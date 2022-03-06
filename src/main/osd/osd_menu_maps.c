#include "osd_menu_maps.h"
#include "filter.h"
#include "profile.h"
#include "stdio.h"
#include "vtx.h"

#define MENU_SIZE(menu) const uint32_t menu##_size = (sizeof(menu) / sizeof(osd_label_t));

extern profile_t profile;
extern vtx_settings_t vtx_settings;
extern vtx_settings_t vtx_settings_copy;

// main menu maps
const osd_label_t main_menu_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "MENU",
        .pos = {13, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "VTX",
        .pos = {7, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PIDS",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FILTERS",
        .pos = {7, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RATES",
        .pos = {7, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FLIGHT MODES",
        .pos = {7, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "OSD ELEMENTS",
        .pos = {7, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SPECIAL FEATURES",
        .pos = {7, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RC LINK",
        .pos = {7, 10},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {7, 11},
    },
};
MENU_SIZE(main_menu_labels);

const uint8_t main_menu_map[] = {OSD_SCREEN_VTX, OSD_SCREEN_PID_PROFILE, OSD_SCREEN_FILTERS, OSD_SCREEN_RATE_PROFILES, OSD_SCREEN_FLIGHT_MODES, OSD_SCREEN_ELEMENTS, OSD_SCREEN_SPECIAL_FEATURES, OSD_SCREEN_RC_LINK};

// pid profiles submenu map
const osd_label_t pid_profiles_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "PID PROFILES",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PID PROFILE 1",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PID PROFILE 2",
        .pos = {7, 5},
    },
};
MENU_SIZE(pid_profiles_labels);

const uint8_t pid_submenu_map[] = {OSD_SCREEN_PID, OSD_SCREEN_PID}; // describes the menu case to call next for each submenu option

// adjust increments used in vector adjust functions
const float bf_pids_increments[] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
const float sw_rates_increments[] = {10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
const float rounded_increments[] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

// pids profiles map
const osd_label_t pid_profile1_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "PID PROFILE 1",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ROLL",
        .pos = {10, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PITCH",
        .pos = {16, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "YAW",
        .pos = {23, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KP",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KI",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KD",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(pid_profile1_labels);

const osd_label_t pid_profile2_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "PID PROFILE 2",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ROLL",
        .pos = {10, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PITCH",
        .pos = {16, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "YAW",
        .pos = {23, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KP",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KI",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "KD",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(pid_profile2_labels);

const uint8_t pid_profile_data_index[9][2] = {
    {1, 0},
    {1, 1},
    {1, 2},
    {2, 0},
    {2, 1},
    {2, 2},
    {3, 0},
    {3, 1},
    {3, 2},
};
const uint8_t pid_profile_grid[9][2] = {
    {1, 1},
    {2, 1},
    {3, 1},
    {1, 2},
    {2, 2},
    {3, 2},
    {1, 3},
    {2, 3},
    {3, 3},
};
const uint8_t pid_profile_data_positions[9][2] = {
    {10, 6},
    {16, 6},
    {22, 6},
    {10, 7},
    {16, 7},
    {22, 7},
    {10, 8},
    {16, 8},
    {22, 8},
};
const float pid_profile_adjust_limits[9][2] = {
    {0.0, 400.0},
    {0.0, 400.0},
    {0.0, 400.0},
    {0.0, 100.0},
    {0.0, 100.0},
    {0.0, 100.0},
    {0.0, 120.0},
    {0.0, 120.0},
    {0.0, 120.0},
};

// filters submenu map
const osd_label_t filter_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "FILTERS",
        .pos = {11, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "GYRO",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "D-TERM",
        .pos = {7, 5},
    },
};
MENU_SIZE(filter_labels);

const uint8_t filter_submenu_map[2] = {OSD_SCREEN_GYRO_FILTER, OSD_SCREEN_DTERM_FILTER};

// flight modes map
uint8_t *flight_modes_ptr[10] = {
    &profile.receiver.aux[AUX_ARMING],
    &profile.receiver.aux[AUX_IDLE_UP],
    &profile.receiver.aux[AUX_LEVELMODE],
    &profile.receiver.aux[AUX_RACEMODE],
    &profile.receiver.aux[AUX_HORIZON],
    &profile.receiver.aux[AUX_STICK_BOOST_PROFILE],
    &profile.receiver.aux[AUX_BUZZER_ENABLE],
    &profile.receiver.aux[AUX_TURTLE],
    &profile.receiver.aux[AUX_MOTOR_TEST],
    &profile.receiver.aux[AUX_FPV_SWITCH],
};
const osd_label_t flight_modes_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "FLIGHT MODES",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "ARMING",
        .pos = {4, 2},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "IDLE UP",
        .pos = {4, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "LEVELMODE",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RACEMODE",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "HORIZON",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STICK BOOST",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "BUZZER",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TURTLE",
        .pos = {4, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MOTOR TEST",
        .pos = {4, 10},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FPV SWITCH",
        .pos = {4, 11},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(flight_modes_labels);

const uint8_t flight_modes_data_positions[10][2] = {
    {17, 2},
    {17, 3},
    {17, 4},
    {17, 5},
    {17, 6},
    {17, 7},
    {17, 8},
    {17, 9},
    {17, 10},
    {17, 11},
};
const uint8_t flight_modes_aux_limits[] = {
    11,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
}; // from aux_channel_t
const uint8_t flight_modes_aux_items[] = {
    AUX_ARMING,
    AUX_IDLE_UP,
    AUX_LEVELMODE,
    AUX_RACEMODE,
    AUX_HORIZON,
    AUX_STICK_BOOST_PROFILE,
    AUX_BUZZER_ENABLE,
    AUX_TURTLE,
    AUX_MOTOR_TEST,
    AUX_FPV_SWITCH,
}; // from aux_function_t
const uint8_t flight_modes_grid[10][2] = {
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

// osd elements submenu map
const osd_label_t osd_elements_menu_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "OSD ELEMENTS",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "ADD OR REMOVE",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EDIT POSITIONS",
        .pos = {7, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EDIT TEXT STYLE",
        .pos = {7, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "EDIT CALLSIGN",
        .pos = {7, 7},
    },
};
MENU_SIZE(osd_elements_menu_labels);

const uint8_t osd_elements_map[] = {
    OSD_SCREEN_ELEMENTS_ADD_REMOVE,
    OSD_SCREEN_ELEMENTS_POSITION,
    OSD_SCREEN_ELEMENTS_STYLE,
    OSD_SCREEN_CALLSIGN,
};

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

// vtx not configured map
const osd_label_t vtx_na_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "VTX CONTROLS",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "SMART AUDIO",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "NOT CONFIGURED",
        .pos = {7, 5},
    },
};
MENU_SIZE(vtx_na_labels);

// vtx menu map
uint8_t *vtx_ptr[4] = {
    &vtx_settings_copy.band,
    &vtx_settings_copy.channel,
    &vtx_settings_copy.power_level,
    &vtx_settings_copy.pit_mode,
};
const osd_label_t vtx_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "VTX CONTROLS",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "BAND",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "CHANNEL",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "POWER LEVEL",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PITMODE",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(vtx_labels);

const uint8_t vtx_data_positions[4][2] = {
    {20, 4},
    {20, 5},
    {20, 6},
    {20, 7},
};
const uint8_t vtx_limits[4] = {4, 7, 3, 1};
const uint8_t vtx_grid[4][2] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
};

// special features menu map
const osd_label_t special_features_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "SPECIAL FEATURES",
        .pos = {7, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STICK BOOST",
        .pos = {7, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MOTOR BOOST",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PID MODIFIERS",
        .pos = {7, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "DIGITAL IDLE",
        .pos = {7, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "LOW BATTERY",
        .pos = {7, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "LEVEL MODE",
        .pos = {7, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TURTLE THROTTLE",
        .pos = {7, 9},
    },
};
MENU_SIZE(special_features_labels);

const uint8_t special_features_map[] = {OSD_SCREEN_STICK_BOOST, OSD_SCREEN_MOTOR_BOOST, OSD_SCREEN_PID_MODIFIER, OSD_SCREEN_DIGITAL_IDLE, OSD_SCREEN_LOWBAT, OSD_SCREEN_LEVEL_MODE, OSD_SCREEN_TURTLE_THROTTLE};

// stick boost submenu map
const osd_label_t stickboost_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "STICK BOOST PROFILES",
        .pos = {5, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "AUX OFF PROFILE 1",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "AUX ON  PROFILE 2",
        .pos = {7, 5},
    },
};
MENU_SIZE(stickboost_labels);

const uint8_t stickboost_submenu_map[] = {OSD_SCREEN_STICK_BOOST_ADJUST, OSD_SCREEN_STICK_BOOST_ADJUST};

// stick boost map
const osd_label_t stickboost1_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "BOOST PROFILE 1",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ROLL",
        .pos = {14, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PITCH",
        .pos = {19, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "YAW",
        .pos = {25, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "ACCELERATOR",
        .pos = {2, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TRANSITION",
        .pos = {2, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(stickboost1_labels);

const osd_label_t stickboost2_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "BOOST PROFILE 2",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "ROLL",
        .pos = {14, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "PITCH",
        .pos = {19, 4},
    },
    {
        .type = OSD_LABEL_INACTIVE,
        .text = "YAW",
        .pos = {25, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "ACCELERATOR",
        .pos = {2, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TRANSITION",
        .pos = {2, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(stickboost2_labels);

const uint8_t stickboost_data_index[6][2] = {
    {1, 0},
    {1, 1},
    {1, 2},
    {2, 0},
    {2, 1},
    {2, 2},
};
const uint8_t stickboost_grid[6][2] = {
    {1, 1},
    {2, 1},
    {3, 1},
    {1, 2},
    {2, 2},
    {3, 2},
};
const uint8_t stickboost_data_positions[9][2] = {
    {13, 6},
    {18, 6},
    {23, 6},
    {13, 8},
    {18, 8},
    {23, 8},
};
const float stickboost_adjust_limits[6][2] = {
    {0, 3.0},
    {0, 3.0},
    {0, 3.0},
    {-1.0, 1.0},
    {-1.0, 1.0},
    {-1.0, 1.0},
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

// levelmode submenu map
const osd_label_t level_submenu_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "LEVEL MODE",
        .pos = {10, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "MAX ANGLE",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "LEVEL STRENGTH",
        .pos = {7, 5},
    },
};
MENU_SIZE(level_submenu_labels);

const uint8_t level_submenu_map[] = {OSD_SCREEN_LEVEL_MAX_ANGLE, OSD_SCREEN_LEVEL_STRENGTH};

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

// motor boost menu map
const osd_label_t motor_boost_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "MOTOR BOOST TYPES",
        .pos = {7, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TORQUE BOOST",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "THROTTLE BOOST",
        .pos = {7, 5},
    },
};
MENU_SIZE(motor_boost_labels);

const uint8_t motor_boost_map[] = {OSD_SCREEN_TORQUE_BOOST, OSD_SCREEN_THROTTLE_BOOST};

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

// gyro filter map
float pointer_redirect = POINTER_REDIRECT;
float *gyrofilter_ptr[4] = {
    &pointer_redirect,
    &profile.filter.gyro[0].cutoff_freq,
    &pointer_redirect,
    &profile.filter.gyro[1].cutoff_freq,
};
uint8_t *gyrofilter_ptr2[4] = {
    &profile.filter.gyro[0].type,
    0,
    &profile.filter.gyro[1].type,
    0,
};
const uint8_t gyrofilter_reboot_request[4] = {
    1,
    0,
    1,
    0,
};
const osd_label_t gyrofilter_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "GYRO FILTERS",
        .pos = {9, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 1 TYPE",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 1 FREQ",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 2 TYPE",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 2 FREQ",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(gyrofilter_labels);

const char gyrofilter_type_labels[3][21] = {
    "NONE",
    " PT1",
    " PT2",
};
const uint8_t gyrofilter_grid[4][2] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
};
const uint8_t gyrofilter_data_positions[4][2] = {
    {18, 4},
    {18, 5},
    {18, 6},
    {18, 7},
};
const float gyrofilter_adjust_limits[4][2] = {
    {0, 2},
    {50, 500},
    {0, 2},
    {50, 500},
};

// dterm filter map
float *dtermfilter_ptr[7] = {
    &pointer_redirect,
    &profile.filter.dterm[0].cutoff_freq,
    &pointer_redirect,
    &profile.filter.dterm[1].cutoff_freq,
    &pointer_redirect,
    &profile.filter.dterm_dynamic_min,
    &profile.filter.dterm_dynamic_max,
};
uint8_t *dtermfilter_ptr2[7] = {
    &profile.filter.dterm[0].type,
    0,
    &profile.filter.dterm[1].type,
    0,
    &profile.filter.dterm_dynamic_enable,
    0,
    0,
};
const uint8_t dtermfilter_reboot_request[7] = {
    1,
    0,
    1,
    0,
    1,
    0,
    0,
};
const osd_label_t dtermfilter_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "D-TERM FILTERS",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 1 TYPE",
        .pos = {4, 3},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 1 FREQ",
        .pos = {4, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 2 TYPE",
        .pos = {4, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "PASS 2 FREQ",
        .pos = {4, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "DYNAMIC",
        .pos = {4, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FREQ MIN",
        .pos = {4, 8},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "FREQ MAX",
        .pos = {4, 9},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {4, 14},
    },
};
MENU_SIZE(dtermfilter_labels);

const char dtermfilter_type_labels[3][21] = {
    "NONE",
    " PT1",
    " PT2",
};
const uint8_t dtermfilter_grid[7][2] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
    {1, 5},
    {1, 6},
    {1, 7},
};
const uint8_t dtermfilter_data_positions[7][2] = {
    {18, 3},
    {18, 4},
    {18, 5},
    {18, 6},
    {18, 7},
    {18, 8},
    {18, 9},
};
const float dtermfilter_adjust_limits[7][2] = {
    {0, 2},
    {50, 500},
    {0, 2},
    {50, 500},
    {0, 1},
    {50, 500},
    {50, 500},
};

// pid modifiers map
float *pidmodify_ptr[4] = {
    &pointer_redirect,
    &pointer_redirect,
    &profile.pid.throttle_dterm_attenuation.tda_breakpoint,
    &profile.pid.throttle_dterm_attenuation.tda_percent,
};
uint8_t *pidmodify_ptr2[4] = {
    &profile.voltage.pid_voltage_compensation,
    &profile.pid.throttle_dterm_attenuation.tda_active,
    0,
    0,
};
const uint8_t pidmodify_reboot_request[4] = {
    0,
    0,
    0,
    0,
};
const osd_label_t pidmodify_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "PID MODIFIERS",
        .pos = {8, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "P-TERM VOLTAGE COMP",
        .pos = {2, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "THROTTLE D ATTENUATE",
        .pos = {2, 5},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TDA BREAKPOINT",
        .pos = {2, 6},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "TDA PERCENT",
        .pos = {2, 7},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "SAVE AND EXIT",
        .pos = {2, 14},
    },
};
MENU_SIZE(pidmodify_labels);

const char pidmodify_type_labels[2][21] = {
    " NONE ",
    "ACTIVE",
};
const uint8_t pidmodify_grid[4][2] = {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
};
const uint8_t pidmodify_data_positions[4][2] = {
    {23, 4},
    {23, 5},
    {23, 6},
    {23, 7},
};
const float pidmodify_adjust_limits[4][2] = {
    {0, 1},
    {0, 1},
    {0, 1},
    {0, 1},
};

// rc link map
const osd_label_t rc_link_labels[] = {
    {
        .type = OSD_LABEL_HEADER,
        .text = "RC LINK",
        .pos = {12, 1},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "STICK CALIBRATION",
        .pos = {7, 4},
    },
    {
        .type = OSD_LABEL_ACTIVE,
        .text = "RSSI SOURCE",
        .pos = {7, 5},
    },
};
MENU_SIZE(rc_link_labels);

const uint8_t rc_link_map[] = {OSD_SCREEN_STICK_WIZARD, OSD_SCREEN_RSSI};

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