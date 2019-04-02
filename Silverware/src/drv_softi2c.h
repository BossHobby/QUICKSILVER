#include <inttypes.h>


void softi2c_init(void);
void softi2c_readdata(int device_address ,int register_address , int *data, int size );
void softi2c_writedata(int device_address ,int register_address , int *data, int size );

int softi2c_read(int device_address , int register_address);
int softi2c_write( int device_address , int address,int value);








