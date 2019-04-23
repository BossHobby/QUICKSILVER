#include "config.h"

#include "project.h"
#include "drv_fmc.h"

#ifdef F0
// address 32768 - 1024 = 31k - last flash block
#define FLASH_ADDR 0x08007C00
#endif
#ifdef F405
/*
Sector 0    0x08000000 - 0x08003FFF 16 Kbytes
Sector 1    0x08004000 - 0x08007FFF 16 Kbytes
Sector 2    0x08008000 - 0x0800BFFF 16 Kbytes
Sector 3    0x0800C000 - 0x0800FFFF 16 Kbytes
Sector 4    0x08010000 - 0x0801FFFF 64 Kbytes
Sector 5    0x08020000 - 0x0803FFFF 128 Kbytes
Sector 6    0x08040000 - 0x0805FFFF 128 Kbytes
Sector 7    0x08060000 - 0x0807FFFF 128 Kbytes
Sector 8    0x08080000 - 0x0809FFFF 128 Kbytes
Sector 9    0x080A0000 - 0x080BFFFF 128 Kbytes
Sector 10   0x080C0000 - 0x080DFFFF 128 Kbytes
Sector 11   0x080E0000 - 0x080FFFFF 128 Kbytes
*/
#define FLASH_ADDR 0x0800C000  //sector 3 address
#endif

extern void failloop( int);

// address 0 - 255 ( for current block FLASH_ADDR )
// value 32 bit to write
void writeword( unsigned long address, unsigned long value)
{
	int test = FLASH_ProgramWord( FLASH_ADDR + (address<<2), value);	
	if ( test != FLASH_COMPLETE )
	{
		FLASH_Lock();
		failloop(5);
	}
}

void fmc_write_float(unsigned long address, float float_to_write) {
	writeword(address, *(unsigned long *) &float_to_write);
}

float fmc_read_float(unsigned long address) {
	unsigned long result = fmc_read(address);
	return *(float*)&result;
}

void fmc_unlock() {
	FLASH_Unlock();
}

void fmc_lock() {
	FLASH_Lock();
}

int fmc_erase( void )
{	
	#ifdef F405
	int test = FLASH_EraseSector( FLASH_Sector_3, VoltageRange_3);
	#endif	
	
	#ifdef F0
	int test = FLASH_ErasePage( FLASH_ADDR );
	#endif
	if ( test != FLASH_COMPLETE ) FLASH_Lock();
    else return 0;
	return 1;// error occured
}

// reads 32 bit value
unsigned long fmc_read(unsigned long address)
{
	address = address * 4 + FLASH_ADDR;
	unsigned int *addressptr = (unsigned int *)address;
	return (*addressptr);
}


