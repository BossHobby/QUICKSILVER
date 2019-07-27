
#include "drv_cc2500.h"

void frsky_init() {
  cc25000_reset();

  cc2500_write_weg(CC2500_IOCFG0, 0x01);
  cc2500_write_weg(CC2500_MCSM1, 0x0C);
  cc2500_write_weg(CC2500_MCSM0, 0x18);
  cc2500_write_weg(CC2500_PKTCTRL1, 0x04);
  cc2500_write_weg(CC2500_PATABLE, 0xFF);
  cc2500_write_weg(CC2500_FSCTRL0, 0x00);
  cc2500_write_weg(CC2500_FREQ2, 0x5C);
  cc2500_write_weg(CC2500_FREQ1, 0x76);
  cc2500_write_weg(CC2500_FREQ0, 0x27);
  cc2500_write_weg(CC2500_MDMCFG1, 0x23);
  cc2500_write_weg(CC2500_MDMCFG0, 0x7A);
  cc2500_write_weg(CC2500_FOCCFG, 0x16);
  cc2500_write_weg(CC2500_BSCFG, 0x6C);
  cc2500_write_weg(CC2500_AGCCTRL2, 0x03);
  cc2500_write_weg(CC2500_AGCCTRL1, 0x40);
  cc2500_write_weg(CC2500_AGCCTRL0, 0x91);
  cc2500_write_weg(CC2500_FREND1, 0x56);
  cc2500_write_weg(CC2500_FREND0, 0x10);
  cc2500_write_weg(CC2500_FSCAL3, 0xA9);
  cc2500_write_weg(CC2500_FSCAL2, 0x0A);
  cc2500_write_weg(CC2500_FSCAL1, 0x00);
  cc2500_write_weg(CC2500_FSCAL0, 0x11);
  cc2500_write_weg(CC2500_FSTEST, 0x59);
  cc2500_write_weg(CC2500_TEST2, 0x88);
  cc2500_write_weg(CC2500_TEST1, 0x31);
  cc2500_write_weg(CC2500_TEST0, 0x0B);
  cc2500_write_weg(CC2500_FIFOTHR, 0x07);
  cc2500_write_weg(CC2500_ADDR, 0x00);

  cc2500_write_reg(CC2500_PKTLEN, 0x19);
  cc2500_write_reg(CC2500_PKTCTRL0, 0x05);
  cc2500_write_reg(CC2500_FSCTRL1, 0x08);
  cc2500_write_reg(CC2500_MDMCFG4, 0xAA);
  cc2500_write_reg(CC2500_MDMCFG3, 0x39);
  cc2500_write_reg(CC2500_MDMCFG2, 0x11);
  cc2500_write_reg(CC2500_DEVIATN, 0x42);
}