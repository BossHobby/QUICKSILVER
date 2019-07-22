#include "drv_spi.h"
#include "project.h"
#include "defines.h"

#ifdef SOFTSPI_NONE
// spi disabled (for pin setting)
int lastbyte = 0;

void spi_init(void)
  {}
void spi_cson(void)
	{}
void spi_csoff(void)
	{}
void spi_sendbyte( int x)
	{ lastbyte = x;}
int spi_sendrecvbyte( int x)
	{ 
	// make it look like a real nrf24 to pass the check
	if ( lastbyte == (0x0f) ) return 0xc6;
	lastbyte = x;
	return 255;}
int spi_sendzerorecvbyte( void )
	{ return 255;}
	
#endif




