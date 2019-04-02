
#include "config.h"
#include "project.h"
#include "drv_fmc.h"


#ifdef FLASH_SAVE2
// flash save to option bytes , 2*8 bit
int flash2_fmc_write( int data1 , int data2)
{
	
    FLASH_Unlock();
    FLASH_OB_Unlock();

	
  FLASH_OB_Erase();

	
	FLASH_OB_ProgramData( 0x1FFFF804, data1 );


	FLASH_OB_ProgramData( 0x1FFFF806, data2 );

	
	FLASH_Lock();
	FLASH_OB_Lock();
	
	return 0;
}


// x = flash2_readdata( OB->DATA0 );
// x = flash2_readdata( OB->DATA1 );
	
int flash2_readdata( unsigned int data )
{
	// checks that data and ~data are valid
	unsigned int userdata = data ;
	int complement = ((userdata &0x0000FF00)>>8 );
	complement |=0xFFFFFF00;

	userdata&=0x000000FF;
	
	if ( userdata!=~complement) 
		return 127;
	
	else return userdata;
}
#endif

