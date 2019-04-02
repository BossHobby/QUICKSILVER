
// soft spi  header file 
//
#include <inttypes.h>

void spi_init(void);
void spi_cson(void);	
void spi_csoff(void);
void spi_sendbyte( int );
int spi_sendrecvbyte( int);
int spi_sendzerorecvbyte( void );



