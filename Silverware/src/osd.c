#include "drv_max7456.h"


void osd_init(void)
{
	spi_max7456_init();
	max7456_init();
	osd_intro();
}






void osd_display(void)
{
//BUILD THIS NEXT
}


