#pragma once

#if defined(RX_SBUS) || defined(RX_DSMX) || defined(RX_DSM2) || defined(RX_CRSF) || defined(RX_IBUS) || defined(RX_FPORT) || defined(RX_UNIFIED_SERIAL)
#define SERIAL_RX
#endif

#if defined(RX_FRSKY_D8) || defined(RX_FRSKY_D16_FCC) || defined(RX_FRSKY_D16_LBT) || defined(RX_REDPINE)
#define RX_FRSKY
#endif