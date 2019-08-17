#include "profile.h"

profile_t profile = {
#ifdef SILVERWARE_RATES
    .rate_mode = RATE_MODE_SILVERWARE,
    .rate = {
        .silverware = {
            .max_rate = {
                MAX_RATE,
                MAX_RATE,
                MAX_RATEYAW,
            },
            .acro_expo = {
                ACRO_EXPO_ROLL,
                ACRO_EXPO_PITCH,
                ACRO_EXPO_YAW,
            },
            .angle_expo = {
                ANGLE_EXPO_ROLL,
                ANGLE_EXPO_PITCH,
                ANGLE_EXPO_YAW,
            },
        },
    },
#endif
#ifdef BETAFLIGHT_RATES
    .rate_mode = RATE_MODE_BETAFLIGHT,
    .rate = {
        .betaflight = {
            .rc_rate = {
                BF_RC_RATE_ROLL,
                BF_RC_RATE_PITCH,
                BF_RC_RATE_YAW,
            },
            .super_rate = {
                BF_SUPER_RATE_ROLL,
                BF_SUPER_RATE_PITCH,
                BF_SUPER_RATE_YAW,
            },
            .expo = {
                BF_EXPO_ROLL,
                BF_EXPO_PITCH,
                BF_EXPO_YAW,
            },
        },
    },
#endif

    .low_rate_mulitplier = LOW_RATES_MULTI,
};