#pragma once

#include "project.h"

#ifdef RX_FRSKY
typedef union {
  struct {
    int8_t offset;
    uint8_t idx;
    uint8_t tx_id[2];
    uint8_t hop_data[50];
    uint8_t rx_num;
    uint8_t _pad;
  };
  uint32_t raw[14];
} frsky_bind_data;
#endif

typedef enum {
  AUX_CHANNEL_0,
  AUX_CHANNEL_1,
  AUX_CHANNEL_2,
  AUX_CHANNEL_3,
  AUX_CHANNEL_4,
  AUX_CHANNEL_5,
  AUX_CHANNEL_6,
  AUX_CHANNEL_7,
  AUX_CHANNEL_8,
  AUX_CHANNEL_9,
  AUX_CHANNEL_10,
  AUX_CHANNEL_11,
  AUX_CHANNEL_12,
  AUX_CHANNEL_ON,
  AUX_CHANNEL_OFF,

  AUX_CHANNEL_MAX
} aux_channel_t;

typedef enum {
  AUX_ARMING,
  AUX_IDLE_UP,
  AUX_LEVELMODE,
  AUX_RACEMODE,
  AUX_HORIZON,
  AUX_STICK_BOOST_PROFILE,
  AUX_TRAVEL_CHECK,
  AUX_RATES,
  AUX_LEDS_ON,
  AUX_BUZZER_ENABLE,
  AUX_STARTFLIP,
  AUX_MOTORS_TO_THROTTLE_MODE,
  AUX_RSSI,
  AUX_FPV_ON,

  AUX_FUNCTION_MAX
} aux_function_t;

uint8_t rx_aux_on(aux_function_t function);
uint8_t rx_auxchange(aux_function_t function);

void rx_init(void);
void checkrx(void);

float rcexpo(float x, float exp);
void rx_apply_expo(void);

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024)
void rx_spektrum_bind(void);
#endif
void usart_invert(void);
