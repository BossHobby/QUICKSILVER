#include <stdio.h>

#include "defines.h"
#include "drv_fmc.h"
#include "drv_rx_serial.h"
#include "drv_serial.h"
#include "drv_time.h"
#include "drv_uart.h"
#include "project.h"
#include "util.h"

#if defined(RX_DSMX_2048) || defined(RX_DSM2_1024)

// global use rx variables
extern float rx[4];
extern char aux[AUX_CHANNEL_MAX];
extern char lastaux[AUX_CHANNEL_MAX];
extern char auxchange[AUX_CHANNEL_MAX];
int failsafe = 0;
int rxmode = 0;
int rx_ready = 0;
int bind_safety = 0;
int rx_bind_enable = 0;

// internal dsm variables

#define DSM_SCALE_PERCENT 147 //adjust this line to match the stick scaling % set in your transmitter
//#define SERIAL_BAUDRATE 115200              //**moved to header file
#define SPEK_FRAME_SIZE 16
#define SPEKTRUM_NEEDED_FRAME_INTERVAL (5000 * (TICK_CLOCK_FREQ_HZ / 1000000))
#define SPEKTRUM_MAX_FADE_PER_SEC 40
#define SPEKTRUM_FADE_REPORTS_PER_SEC 2
#define SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT 12
#define SPEKTRUM_2048_CHANNEL_COUNT 12
#define SPEKTRUM_1024_CHANNEL_COUNT 7
#ifdef RX_DSMX_2048
#define CHANNEL_COUNT 10
#define BIND_PULSES 9
// 11 bit frames
static uint8_t spek_chan_shift = 3;
static uint8_t spek_chan_mask = 0x07;
#endif

#ifdef RX_DSM2_1024
#define CHANNEL_COUNT 7
#define BIND_PULSES 3
// 10 bit frames
static uint8_t spek_chan_shift = 2;
static uint8_t spek_chan_mask = 0x03;
#endif

static uint32_t channels[CHANNEL_COUNT];
static int rcFrameComplete = 0;
int framestarted = -1;
int rx_frame_pending;
int rx_frame_pending_last;
uint32_t flagged_time;
static volatile uint8_t spekFrame[SPEK_FRAME_SIZE];
float dsm2_scalefactor = (0.29354210f / DSM_SCALE_PERCENT);
float dsmx_scalefactor = (0.14662756f / DSM_SCALE_PERCENT);

// Receive ISR callback

void RX_USART_ISR(void) {
  static uint8_t spekFramePosition = 0;

  unsigned long maxticks = SysTick->LOAD;
  unsigned long ticks = SysTick->VAL;
  unsigned long spekTimeInterval;
  static unsigned long lastticks;
  if (ticks < lastticks)
    spekTimeInterval = lastticks - ticks;
  else { // overflow ( underflow really)
    spekTimeInterval = lastticks + (maxticks - ticks);
  }
  lastticks = ticks;

  if (USART_GetFlagStatus(SERIAL_RX_USART, USART_FLAG_ORE)) {
    // overflow means something was lost
    USART_ClearFlag(SERIAL_RX_USART, USART_FLAG_ORE);
  }
  if (spekTimeInterval > SPEKTRUM_NEEDED_FRAME_INTERVAL) {
    spekFramePosition = 0;
  }
  if (spekFramePosition < SPEK_FRAME_SIZE) {
    spekFrame[spekFramePosition++] = USART_ReceiveData(SERIAL_RX_USART);
    if (spekFramePosition < SPEK_FRAME_SIZE) {
      rcFrameComplete = 0;
    } else {
      rcFrameComplete = 1;
    }
  }
  spekFramePosition %= (SPEK_FRAME_SIZE);
}

void spektrumFrameStatus(void) {
  if (rcFrameComplete == 0) {
    rx_frame_pending = 1; //flags when last time through we had a frame and this time we dont
  } else {
    rcFrameComplete = 0; //isr callback triggers alert of fresh data in buffer

    for (int b = 3; b < SPEK_FRAME_SIZE; b += 2) { //stick data in channels buckets
      const uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
      if (spekChannel < CHANNEL_COUNT && spekChannel < SPEKTRUM_MAX_SUPPORTED_CHANNEL_COUNT) {
        channels[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
        framestarted = 1;
        rx_frame_pending = 0; //flags when last time through we didn't have a frame and this time we do
        bind_safety++;        // incriments up as good frames come in till we pass a safe point where aux channels are updated
      }
    }
  }
}

void dsm_init(void) {
  usart_rx_init();  //initialize usart in drv_rx_serial
  framestarted = 0; // set setup complete flag
}

// Send Spektrum bind pulses to a GPIO e.g. TX1
void rx_spektrum_bind(void) {
#ifdef SERIAL_RX_SPEKBIND_RX_PIN
  rx_bind_enable = fmc_read_float(56);
  if (rx_bind_enable == 0) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SERIAL_RX_SPEKBIND_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure);

    // RX line, set high
    GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
    // Bind window is around 20-140ms after powerup
    delay(60000);

    for (uint8_t i = 0; i < BIND_PULSES; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
      // RX line, drive low for 120us
      GPIO_ResetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
      delay(120);

      // RX line, drive high for 120us
      GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_RX_PIN);
      delay(120);
    }
  }
#endif
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = SERIAL_RX_SPEKBIND_BINDTOOL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SERIAL_RX_PORT, &GPIO_InitStructure);

  // RX line, set high
  GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
  // Bind window is around 20-140ms after powerup
  delay(60000);

  for (uint8_t i = 0; i < BIND_PULSES; i++) { // 9 pulses for internal dsmx 11ms, 3 pulses for internal dsm2 22ms
    // RX line, drive low for 120us
    GPIO_ResetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
    delay(120);

    // RX line, drive high for 120us
    GPIO_SetBits(SERIAL_RX_PORT, SERIAL_RX_SPEKBIND_BINDTOOL_PIN);
    delay(120);
  }
}

void rx_init(void) {
}

void checkrx()

{

  if (framestarted < 0) {
    failsafe = 1;          //kill motors while initializing usart (maybe not necessary)
    dsm_init();            // toggles "framestarted = 0;" after initializing
    rxmode = !RXMODE_BIND; // put LEDS in normal signal status
  }

  if (framestarted == 0) { // this is the failsafe condition
    failsafe = 1;          //keeps motors off while waiting for first frame and if no new frame for more than 1s
  }

  rx_frame_pending_last = rx_frame_pending;
  spektrumFrameStatus();
  if (rx_frame_pending != rx_frame_pending_last)
    flagged_time = gettime(); //updates flag to current time only on changes of losing a frame or getting one back
  if (gettime() - flagged_time > FAILSAFETIME)
    framestarted = 0; //watchdog if more than 1 sec passes without a frame causes failsafe

  if (framestarted == 1) {
    if ((bind_safety < 900) && (bind_safety > 0))
      rxmode = RXMODE_BIND; // normal rx mode - removes waiting for bind led leaving failsafe flashes as data starts to come in

      // TAER channel order
#ifdef RX_DSMX_2048
    rx[0] = (channels[1] - 1024.0f) * dsmx_scalefactor;
    rx[1] = (channels[2] - 1024.0f) * dsmx_scalefactor;
    rx[2] = (channels[3] - 1024.0f) * dsmx_scalefactor;
    rx[3] = ((channels[0] - 1024.0f) * dsmx_scalefactor * 0.5f) + 0.5f;

    if (rx[3] > 1)
      rx[3] = 1;
    if (rx[3] < 0)
      rx[3] = 0;
#endif

#ifdef RX_DSM2_1024
    rx[0] = (channels[1] - 512.0f) * dsm2_scalefactor;
    rx[1] = (channels[2] - 512.0f) * dsm2_scalefactor;
    rx[2] = (channels[3] - 512.0f) * dsm2_scalefactor;
    rx[3] = ((channels[0] - 512.0f) * dsm2_scalefactor * 0.5f) + 0.5f;

    if (rx[3] > 1)
      rx[3] = 1;
    if (rx[3] < 0)
      rx[3] = 0;
#endif

    rx_apply_expo();

#ifdef RX_DSMX_2048
    aux[AUX_CHANNEL_0] = (channels[4] > 1100) ? 1 : 0;        //1100 cutoff intentionally selected to force aux channels low if
    aux[AUX_CHANNEL_1] = (channels[5] > 1100) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
    aux[AUX_CHANNEL_2] = (channels[6] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_3] = (channels[7] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_4] = (channels[8] > 1100) ? 1 : 0;
    aux[AUX_CHANNEL_5] = (channels[9] > 1100) ? 1 : 0;
#endif

#ifdef RX_DSM2_1024
    aux[AUX_CHANNEL_0] = (channels[4] > 550) ? 1 : 0;        //550 cutoff intentionally selected to force aux channels low if
    aux[AUX_CHANNEL_1] = (channels[5] > 550) ? 1 : 0; //being controlled by a transmitter using a 3 pos switch in center state
    aux[AUX_CHANNEL_2] = (channels[6] > 550) ? 1 : 0;
#endif

    if (bind_safety > 900) { //requires 10 good frames to come in before rx_ready safety can be toggled to 1.  900 is about 2 seconds of good data
      rx_ready = 1;          // because aux channels initialize low and clear the binding while armed flag before aux updates high
      failsafe = 0;          // turn off failsafe delayed a bit to emmulate led behavior of sbus protocol - optional either here or just above here
      rxmode = !RXMODE_BIND; // restores normal led operation
      bind_safety = 901;     // reset counter so it doesnt wrap
    }
  }
}
#endif
