#include <stdint.h>
#include <unity.h>

extern float bmp280_test_compensate(const uint8_t calibration[24], const uint8_t data[6]);
extern float bmp388_test_compensate(const uint8_t calibration[21], const uint8_t data[6]);

static void put_le16(uint8_t *data, uint16_t value) {
  data[0] = value;
  data[1] = value >> 8;
}

static void bmp280_sample_calibration(uint8_t calibration[24]) {
  // Bosch reference vector, also used by Betaflight's baro_bmp280_unittest.
  const int32_t values[] = {27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
  for (unsigned i = 0; i < 12; i++) {
    put_le16(calibration + 2 * i, values[i]);
  }
}

void test_bmp280_raw_reference_sample_returns_pascals(void) {
  uint8_t calibration[24];
  bmp280_sample_calibration(calibration);
  // Pressure ADC 415148 and temperature ADC 519888, MSB-first 20-bit.
  const uint8_t sample[6] = {0x65, 0x5a, 0xc0, 0x7e, 0xed, 0x00};
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100653.25f, bmp280_test_compensate(calibration, sample));
}

void test_bmp280_discards_unused_sample_nibbles(void) {
  uint8_t calibration[24];
  bmp280_sample_calibration(calibration);
  const uint8_t sample[6] = {0x65, 0x5a, 0xcf, 0x7e, 0xed, 0x0f};
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100653.25f, bmp280_test_compensate(calibration, sample));
}

void test_bmp388_raw_reference_sample_retains_cubic_correction(void) {
  // Betaflight baro_bmp388_unittest sample, including nonzero signed P11.
  uint8_t calibration[21] = {0};
  put_le16(calibration + 0, 27772);
  put_le16(calibration + 2, 18638);
  calibration[4] = (uint8_t)-10;
  put_le16(calibration + 5, 878);
  put_le16(calibration + 7, (uint16_t)-2023);
  calibration[9] = 35;
  put_le16(calibration + 11, 24476);
  put_le16(calibration + 13, 30501);
  calibration[15] = (uint8_t)-13;
  calibration[16] = (uint8_t)-10;
  put_le16(calibration + 17, 16545);
  calibration[19] = 21;
  calibration[20] = (uint8_t)-60;
  const uint32_t pressure = 7323488, temperature = 9937920;
  const uint8_t sample[6] = {(uint8_t)pressure, (uint8_t)(pressure >> 8), (uint8_t)(pressure >> 16), (uint8_t)temperature, (uint8_t)(temperature >> 8), (uint8_t)(temperature >> 16)};
  // Bosch BMP3_SensorAPI floating-point reference equations give 101211.286 Pa.
  // Its integer compensation is scaled by 100, unlike BMP280's Q24.8 result.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 101211.29f, bmp388_test_compensate(calibration, sample));
}
