#include "vtx.h"

#include "drv_gpio.h"
#include "drv_serial_smart_audio.h"
#include "project.h"
#include "rx.h"

#ifdef FPV_ON
// bind / normal rx mode
extern int rxmode;
// failsafe on / off
extern int failsafe;

static int fpv_init = 0;
#endif

void vtx_init() {
#ifdef ENABLE_SMART_AUDIO
  serial_smart_audio_init();
  serial_smart_audio_send_payload(SA_CMD_GET_SETTINGS, NULL, 0);
#endif
}

void vtx_update() {
#ifdef FPV_ON
  if (rx_aux_on(AUX_FPV_ON)) {
    // fpv switch on
    if (!fpv_init && rxmode == RXMODE_NORMAL) {
      fpv_init = gpio_init_fpv();
    }
    if (fpv_init) {
      GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_SET);
    }
  } else {
    // fpv switch off
    if (fpv_init) {
      if (failsafe) {
        GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_SET);
      } else {
        GPIO_WriteBit(FPV_PORT, FPV_PIN, Bit_RESET);
      }
    }
  }
#endif
}