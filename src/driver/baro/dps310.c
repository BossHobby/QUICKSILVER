#include "driver/baro/dps310.h"

#include "driver/i2c.h"

#ifdef USE_BARO

#define DPS310_I2C_ADDR 0x76

#define DPS310_REG_PSR_B2 0x00
#define DPS310_REG_PSR_B1 0x01
#define DPS310_REG_PSR_B0 0x02
#define DPS310_REG_TMP_B2 0x03
#define DPS310_REG_TMP_B1 0x04
#define DPS310_REG_TMP_B0 0x05
#define DPS310_REG_PRS_CFG 0x06
#define DPS310_REG_TMP_CFG 0x07
#define DPS310_REG_MEAS_CFG 0x08
#define DPS310_REG_CFG_REG 0x09

#define DPS310_REG_RESET 0x0C
#define DPS310_REG_ID 0x0D

#define DPS310_REG_COEF 0x10
#define DPS310_REG_COEF_SRCE 0x28

#define DPS310_ID_REV_AND_PROD_ID (0x10) // Infineon DPS310
#define SPL07_003_CHIP_ID (0x11)         // SPL07_003

#define DPS310_RESET_BIT_SOFT_RST (0x09) // 0b1001

#define DPS310_MEAS_CFG_COEF_RDY (1 << 7)
#define DPS310_MEAS_CFG_SENSOR_RDY (1 << 6)
#define DPS310_MEAS_CFG_TMP_RDY (1 << 5)
#define DPS310_MEAS_CFG_PRS_RDY (1 << 4)
#define DPS310_MEAS_CFG_MEAS_CTRL_CONT (0x7)

#define DPS310_PRS_CFG_BIT_PM_RATE_32HZ (0x50) //  101 - 32 measurements pr. sec.
#define DPS310_PRS_CFG_BIT_PM_PRC_16 (0x04)    // 0100 - 16 times (Standard).

#define DPS310_TMP_CFG_BIT_TMP_EXT (0x80)       //
#define DPS310_TMP_CFG_BIT_TMP_RATE_32HZ (0x50) //  101 - 32 measurements pr. sec.
#define DPS310_TMP_CFG_BIT_TMP_PRC_16 (0x04)    // 0100 - 16 times (Standard).

#define DPS310_CFG_REG_BIT_P_SHIFT (0x04)
#define DPS310_CFG_REG_BIT_T_SHIFT (0x08)

#define DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE (0x80)

extern i2c_bus_device_t baro_bus;
typedef struct {
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
  int16_t c31;
  int16_t c40;
} dps310_coeffs_t;

static uint8_t chip_id = 0;
static dps310_coeffs_t coeffs;

static int32_t get_twos_complement(uint32_t raw, uint8_t length) {
  if (raw & ((int)1 << (length - 1))) {
    return ((int32_t)raw) - ((int32_t)1 << length);
  } else {
    return raw;
  }
}

static void dps310_set_bits(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t setbits) {
  uint8_t val = i2c_read_reg(&baro_bus, reg);
  if ((val & setbits) != setbits) {
    val |= setbits;
    i2c_write_reg(&baro_bus, reg, val);
  }
}

static baro_types_t dps310_init() {
  baro_bus.address = DPS310_I2C_ADDR;

  chip_id = i2c_read_reg(&baro_bus, DPS310_REG_ID);
  if (chip_id != DPS310_ID_REV_AND_PROD_ID && chip_id != SPL07_003_CHIP_ID) {
    return BARO_TYPE_INVALID;
  }

  i2c_write_reg(&baro_bus, DPS310_REG_RESET, DPS310_RESET_BIT_SOFT_RST);
  time_delay_ms(40);

  while (true) {
    const uint8_t status = i2c_read_reg(&baro_bus, DPS310_REG_MEAS_CFG);
    if ((status & DPS310_MEAS_CFG_COEF_RDY) && (status & DPS310_MEAS_CFG_SENSOR_RDY)) {
      break;
    }
    time_delay_ms(1);
  }

  const uint32_t coeff_length = chip_id == SPL07_003_CHIP_ID ? 21 : 18;
  uint8_t coeff[coeff_length];
  for (uint32_t i = 0; i < coeff_length; i++) {
    coeff[i] = i2c_read_reg(&baro_bus, DPS310_REG_COEF + i);
  }

  coeffs.c0 = get_twos_complement(((uint32_t)coeff[0] << 4) | (((uint32_t)coeff[1] >> 4) & 0x0F), 12);
  coeffs.c1 = get_twos_complement((((uint32_t)coeff[1] & 0x0F) << 8) | (uint32_t)coeff[2], 12);
  coeffs.c00 = get_twos_complement(((uint32_t)coeff[3] << 12) | ((uint32_t)coeff[4] << 4) | (((uint32_t)coeff[5] >> 4) & 0x0F), 20);
  coeffs.c10 = get_twos_complement((((uint32_t)coeff[5] & 0x0F) << 16) | ((uint32_t)coeff[6] << 8) | (uint32_t)coeff[7], 20);
  coeffs.c01 = get_twos_complement(((uint32_t)coeff[8] << 8) | (uint32_t)coeff[9], 16);
  coeffs.c11 = get_twos_complement(((uint32_t)coeff[10] << 8) | (uint32_t)coeff[11], 16);
  coeffs.c20 = get_twos_complement(((uint32_t)coeff[12] << 8) | (uint32_t)coeff[13], 16);
  coeffs.c21 = get_twos_complement(((uint32_t)coeff[14] << 8) | (uint32_t)coeff[15], 16);
  coeffs.c30 = get_twos_complement(((uint32_t)coeff[16] << 8) | (uint32_t)coeff[17], 16);
  if (chip_id == SPL07_003_CHIP_ID) {
    coeffs.c31 = get_twos_complement(((uint32_t)coeff[18] << 4) | (((uint32_t)coeff[19] >> 4) & 0x0F), 12);
    coeffs.c40 = get_twos_complement((((uint32_t)coeff[19] & 0x0F) << 8) | (uint32_t)coeff[20], 12);
  } else {
    coeffs.c31 = 0;
    coeffs.c40 = 0;
  }

  // PRS_CFG: pressure measurement rate (32 Hz) and oversampling (16 time standard)
  dps310_set_bits(&baro_bus, DPS310_REG_PRS_CFG, DPS310_PRS_CFG_BIT_PM_RATE_32HZ | DPS310_PRS_CFG_BIT_PM_PRC_16);

  // TMP_CFG: temperature measurement rate (32 Hz) and oversampling (16 times)
  if (chip_id == SPL07_003_CHIP_ID) {
    dps310_set_bits(&baro_bus, DPS310_REG_TMP_CFG, DPS310_TMP_CFG_BIT_TMP_RATE_32HZ | DPS310_TMP_CFG_BIT_TMP_PRC_16);
  } else {
    const uint8_t temp_coef_source = i2c_read_reg(&baro_bus, DPS310_REG_COEF_SRCE);
    dps310_set_bits(&baro_bus, DPS310_REG_TMP_CFG, DPS310_TMP_CFG_BIT_TMP_RATE_32HZ | DPS310_TMP_CFG_BIT_TMP_PRC_16 | (temp_coef_source & DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE));
  }

  // CFG_REG: set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
  dps310_set_bits(&baro_bus, DPS310_REG_CFG_REG, DPS310_CFG_REG_BIT_T_SHIFT | DPS310_CFG_REG_BIT_P_SHIFT);

  // MEAS_CFG: Continuous pressure and temperature measurement
  dps310_set_bits(&baro_bus, DPS310_REG_MEAS_CFG, DPS310_MEAS_CFG_MEAS_CTRL_CONT);

  return BARO_TYPE_DPS310;
}

static bool dps310_get_pressure(float *pressure) {
  static uint8_t status = 0;
  if (!i2c_read_async(&baro_bus, DPS310_REG_MEAS_CFG, &status, 1)) {
    return false;
  }

  const bool sample_ready = status & (DPS310_MEAS_CFG_PRS_RDY | DPS310_MEAS_CFG_TMP_RDY);
  if (!sample_ready) {
    return false;
  }

  static const float kT = 253952; // 16 times (Standard)
  static const float kP = 253952; // 16 times (Standard)

  const int32_t Praw = get_twos_complement((baro_buf[0] << 16) + (baro_buf[1] << 8) + baro_buf[2], 24);
  const int32_t Traw = get_twos_complement((baro_buf[3] << 16) + (baro_buf[4] << 8) + baro_buf[5], 24);

  const float Praw_sc = Praw / kP;
  const float Traw_sc = Traw / kT;

  const float c00 = coeffs.c00;
  const float c01 = coeffs.c01;
  const float c10 = coeffs.c10;
  const float c11 = coeffs.c11;
  const float c20 = coeffs.c20;
  const float c21 = coeffs.c21;
  const float c30 = coeffs.c30;
  const float c31 = coeffs.c31;
  const float c40 = coeffs.c40;

  // See section 4.9.1, How to Calculate Compensated Pressure Values, of datasheet
  if (chip_id == SPL07_003_CHIP_ID) {
    *pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * (c30 + Praw_sc * c40))) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * (c21 + Praw_sc * c31));
  } else {
    *pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);
  }

  i2c_read_reg_bytes(&baro_bus, DPS310_REG_PSR_B2, baro_buf, sizeof(baro_buf));

  return true;
}

baro_interface_t dps310_interface = {
    .init = dps310_init,
    .get_pressure = dps310_get_pressure,
};
#endif