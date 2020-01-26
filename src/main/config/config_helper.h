#pragma once

#if defined(RX_SBUS) || defined(RX_DSMX_2048) || defined(RX_DSM2_1024) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
#define SERIAL_RX
#endif

#if defined(RX_FRSKY_D8) || defined(RX_FRSKY_D16_FCC) || defined(RX_FRSKY_D16_LBT)
#define RX_FRSKY
#endif