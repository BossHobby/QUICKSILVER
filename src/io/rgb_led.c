#include <math.h>

#include "driver/rgb_led.h"
#include "driver/time.h"
#include "flight/control.h"
#include "project.h"
#include "util/util.h"

extern int ledcommand;

// normal flight rgb colour - LED switch ON
#define RGB_VALUE_INFLIGHT_ON RGB(255, 255, 255)

// normal flight rgb colour - LED switch OFF
#define RGB_VALUE_INFLIGHT_OFF RGB(0, 0, 0)

//  colour before bind
#define RGB_VALUE_BEFORE_BIND RGB(0, 128, 128)

// fade from one color to another when changed
#define RGB_FILTER_ENABLE
#define RGB_FILTER_TIME_MICROSECONDS 50e3

// runs the update once every 16 loop times ( 16 mS )
#define DOWNSAMPLE 16

#define RGB_FILTER_TIME FILTERCALC(1000 * DOWNSAMPLE, RGB_FILTER_TIME_MICROSECONDS)
#define RGB(r, g, b) ((((int)g & 0xff) << 16) | (((int)r & 0xff) << 8) | ((int)b & 0xff))

#if (RGB_LED_NUMBER > 0)

// array with individual led brightnesses
int rgb_led_value[RGB_LED_NUMBER];
// loop count for downsampling
int rgb_loopcount = 0;
// rgb low pass filter variables
float r_filt, g_filt, b_filt;

// sets all leds to a brightness
void rgb_led_set_all(int rgb) {
#ifdef RGB_FILTER_ENABLE
  // deconstruct the colour into components
  int g = rgb >> 16;
  int r = (rgb & 0x0000FF00) >> 8;
  int b = rgb & 0xff;

  // filter individual colors
  lpf(&r_filt, r, RGB_FILTER_TIME);
  lpf(&g_filt, g, RGB_FILTER_TIME);
  lpf(&b_filt, b, RGB_FILTER_TIME);

  int temp = RGB(r_filt, g_filt, b_filt);

  for (int i = 0; i < RGB_LED_NUMBER; i++)
    rgb_led_value[i] = temp;

#else
  for (int i = 0; i < RGB_LED_NUMBER; i++)
    rgb_led_value[i] = rgb;
#endif
}

// set an individual led brightness
void rgb_led_set_one(int led_number, int rgb) {
  rgb_led_value[led_number] = rgb;
}

// flashes between 2 colours, duty cycle 1 - 15
void rgb_ledflash(int color1, int color2, uint32_t period, int duty) {
  if (time_micros() % period > (period * duty) >> 4) {
    rgb_led_set_all(color1);
  } else {
    rgb_led_set_all(color2);
  }
}

// speed of movement
float KR_SPEED = 0.005f * DOWNSAMPLE;

float kr_position = 0;
int kr_dir = 0;

// knight rider style led movement
void rgb_knight_rider() {
  if (kr_dir) {
    kr_position += KR_SPEED;
    if (kr_position > RGB_LED_NUMBER - 1)
      kr_dir = !kr_dir;
  } else {
    kr_position -= KR_SPEED;
    if (kr_position < 0)
      kr_dir = !kr_dir;
  }

  // calculate led value
  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    float led_bright = fabsf((float)i - kr_position);
    if (led_bright > 1.0f)
      led_bright = 1.0f;
    led_bright = 1.0f - led_bright;

    // set a green background as well, 32 brightness
    rgb_led_set_one(i, RGB((led_bright * 255.0f), (32.0f - led_bright * 32.0f), 0));
  }
}

// 2 led flasher
void rgb_ledflash_twin(int color1, int color2, uint32_t period) {
  if (time_micros() % period > (period / 2)) {
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      if ((i / 2) * 2 == i)
        rgb_led_set_one(i, color1);
      else
        rgb_led_set_one(i, color2);
    }
  } else {
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      if ((i / 2) * 2 == i)
        rgb_led_set_one(i, color2);
      else
        rgb_led_set_one(i, color1);
    }
  }
}

// main function
void rgb_led_lvc() {
  rgb_loopcount++;
  if (rgb_loopcount > DOWNSAMPLE) {
    rgb_loopcount = 0;
    // led flash logic
    if (flags.lowbatt) {
      // rgb_led_set_all( RGB( 255 , 0 , 0 ) );
      rgb_ledflash(RGB(255, 0, 0), RGB(255, 32, 0), 500000, 8);
    } else {
      if (flags.rx_mode == RXMODE_BIND) {
        // bind mode
        // rgb_ledflash ( RGB( 0 , 0 , 255 ), RGB( 0 , 128 , 0 ), 1000000, 12);
        //	rgb_ledflash_twin( RGB( 0 , 0 , 255 ), RGB( 0 , 128 , 0 ), 1000000);
        rgb_led_set_all(RGB_VALUE_BEFORE_BIND);
        //	rgb_knight_rider();
      } else { // non bind
        if (flags.failsafe) {
          // failsafe flash
          rgb_ledflash(RGB(0, 128, 0), RGB(0, 0, 128), 500000, 8);
          // rgb_led_set_all( RGB( 0 , 128 , 128 ) );
        } else {

          if (rx_aux_on(AUX_LEDS_ON))

            rgb_led_set_all(RGB_VALUE_INFLIGHT_ON);
          else
            rgb_led_set_all(RGB_VALUE_INFLIGHT_OFF);
        }
      }
    }

#ifdef RGB_LED_DMA
    // send dma start signal
    rgb_send(0);
#else
    // send data to leds
    for (int i = 0; i < RGB_LED_NUMBER; i++) {
      rgb_send(rgb_led_value[i]);
    }
#endif
  }
}

#endif
