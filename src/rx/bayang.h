#pragma once

#include "rx/rx.h"

// defines for bayang protocol radio

#if (!defined(USE_DEVO) && !defined(USE_MULTI))
#define USE_DEVO
#endif

#define UNMAPPED_CHANNEL

#ifdef USE_DEVO
// devo tx channel mapping
// also for nr24multipro
#define CH_INV AUX_CHANNEL_0
#define CH_FLIP AUX_CHANNEL_1
#define CH_PIC AUX_CHANNEL_2
#define CH_VID AUX_CHANNEL_3
#define CH_HEADFREE AUX_CHANNEL_4
#define CH_RTH AUX_CHANNEL_5
// #define CH_EXPERT AUX_CHANNEL_6		//Channel 11 / expert comes in set high to deactivate toy type low rates from deviation.  No longer decoded
#define CH_EMG AUX_CHANNEL_7
// #define CH_TO AUX_CHANNEL_8			//Channel 13 is not supported by deviation - no longer decoded
#endif

#ifdef USE_MULTI
// multimodule mapping ( taranis )
#define CH_FLIP AUX_CHANNEL_0
#define CH_RTH AUX_CHANNEL_1
#define CH_PIC AUX_CHANNEL_2
#define CH_VID AUX_CHANNEL_3
#define CH_HEADFREE AUX_CHANNEL_4
#define CH_INV AUX_CHANNEL_5
// #define CH_EXPERT AUX_CHANNEL_6		//multimodule channel behavior unknown
#define CH_EMG AUX_CHANNEL_7 // multimodule channel behavior unknown
// #define CH_TO AUX_CHANNEL_8			//multimodule channel behavior unknown
#endif

/*	//legacy stuff to sort through later
// trims numbers have to be sequential, start at CH_PIT_TRIM
#define CH_PIT_TRIM 6
#define CH_RLL_TRIM 7
#define CH_THR_TRIM 8
#define CH_YAW_TRIM 9

// defines for cg023 protocol
#define CH_CG023_LED 3
#define CH_CG023_FLIP 0
#define CH_CG023_STILL 2
#define CH_CG023_VIDEO 1

#define CH_H7_FLIP 0
#define CH_H7_VIDEO 1
#define CH_H7_FS 2

#define CH_CX10_CH0 0
#define CH_CX10_CH2 2

#define CH_AUX3 AUX_CHANNEL_OFF
#define CH_AUX4 AUX_CHANNEL_OFF
*/

typedef struct {
  char rfchannel[4];
  char rxaddress[5];
  uint8_t telemetry_enabled;
} rx_bayang_bind_data_t;

void rx_protocol_init();
bool rx_bayang_check();

struct rxdebug {
  uint32_t packettime;
  int failcount;
  int packetpersecond;
  int channelcount[4];
};
