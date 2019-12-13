
#define LEDALL 15

#include <inttypes.h>

void ledon( uint8_t val );
void ledoff( uint8_t val );
void ledset( int val );
void ledflash( uint32_t period , int duty );

void auxledon( uint8_t val );
void auxledoff( uint8_t val );
void auxledflash( uint32_t period , int duty );

uint8_t led_pwm( uint8_t pwmval);




































