#pragma once

#include "rx.h"

// defines for bayang protocol radio
#define CH_FLIP AUX_CHANNEL_0
#define CH_EXPERT AUX_CHANNEL_1
#define CH_HEADFREE AUX_CHANNEL_2
#define CH_RTH AUX_CHANNEL_3

// next 3 channels only when *not* using USE_STOCK_TX
#define CH_INV AUX_CHANNEL_6
#define CH_VID AUX_CHANNEL_7
#define CH_PIC AUX_CHANNEL_8

#define CH_EMG AUX_CHANNEL_10
#define CH_TO AUX_CHANNEL_11

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

#ifdef USE_DEVO
// devo tx channel mapping
// also for nr24multipro
#undef CH_FLIP
#define CH_FLIP AUX_CHANNEL_1
#undef CH_HEADFREE
#define CH_HEADFREE AUX_CHANNEL_4
#undef CH_RTH
#define CH_RTH AUX_CHANNEL_5

#undef CH_INV
#define CH_INV AUX_CHANNEL_0
#undef CH_PIC
#define CH_PIC AUX_CHANNEL_2
#undef CH_VID
#define CH_VID AUX_CHANNEL_3
#endif

#ifdef USE_MULTI
// multimodule mapping ( taranis )
#undef CH_FLIP
#define CH_FLIP AUX_CHANNEL_0
#undef CH_HEADFREE
#define CH_HEADFREE AUX_CHANNEL_4
#undef CH_RTH
#define CH_RTH AUX_CHANNEL_1

#undef CH_INV
#define CH_INV AUX_CHANNEL_5
#undef CH_PIC
#define CH_PIC AUX_CHANNEL_2
#undef CH_VID
#define CH_VID AUX_CHANNEL_3
#endif

void rx_init(void);
void checkrx(void);

struct rxdebug {
  unsigned long packettime;
  int failcount;
  int packetpersecond;
  int channelcount[4];
};
