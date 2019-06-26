void spi_max7456_init(void);
void max7456_init(void);
void osd_intro(void);
void osd_clear(void);

#define DMM 0x04 
#define DMAH 0x05 
#define DMAL 0x06 
#define DMDI 0x07 
//#define CMM 0x08
//#define CMAH 0x09
//#define CMAL 0x0A
//#define CMDI 0x0B
#define VM0 0x00 
#define VM1 0x01 
#define RB0 0x10 
#define STAT 0xA2 //Status register read address
#define CMDO 0xC0 
#define OSDM 0x0C
#define VOS 0x03
//#define DMDO 0XB0 

#define BLINK 0x11
#define INVERT 0x09
#define TEXT 0x01




#define MAX7456_VSYNC 2


// do not change defines below this line
#define PAL 1
#define NTSC 0
#define NONE 2

#define YES 1
#define NO 0

#define VM0_R 0x80
#define OSDBL_R 0xEC
#define OSDBL_W 0x6C

#define BATT1 0   //A2
#define BATT2 1   //A0
#define RSSI 2    // A3
#define CURR 3    // A1
#define VCC 4
#define FIRTEST 7

