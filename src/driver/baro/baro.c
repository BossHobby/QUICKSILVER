#include "baro.h"

#include <math.h>

#include "driver/i2c.h"
#include "flight/control.h"

#include "driver/baro/bmp280.h"
#include "driver/baro/bmp388.h"
#include "driver/baro/dps310.h"

#ifdef USE_BARO

#define P0 101325.0f // Standard pressure at sea level in pascals (Pa)

uint8_t baro_buf[6];
i2c_bus_device_t baro_bus;

static baro_interface_t *baro = NULL;
static baro_types_t baro_type = BARO_TYPE_INVALID;
static baro_interface_t *const baro_interfaces[BARO_TYPE_MAX] = {
    [BARO_TYPE_INVALID] = NULL,
    [BARO_TYPE_BMP280] = &bmp280_interface,
    [BARO_TYPE_BMP388] = &bmp388_interface,
    [BARO_TYPE_DPS310] = &dps310_interface,
};

baro_types_t baro_init() {
  if (target.baro.port == I2C_PORT_INVALID)
    return baro_type;

  baro_bus.port = target.baro.port;
  if (!i2c_bus_device_init(&baro_bus))
    return baro_type;

  for (baro_types_t i = BARO_TYPE_INVALID + 1; i < BARO_TYPE_MAX; i++) {
    baro = baro_interfaces[i];
    baro_type = baro->init();
    if (baro_type != BARO_TYPE_INVALID)
      break;
  }

  return baro_type;
}

static float baro_pressure_to_altitude(const float pressure) {
  return (1.0f - powf(pressure / P0, 1.0f / 5.25588f)) / 2.25577e-5f;
}

bool baro_update() {
  if (baro_type == BARO_TYPE_INVALID)
    return false;

  if (!i2c_is_idle(&baro_bus))
    return false;

  if (!baro->get_pressure(&state.baro_pressure))
    return false;

  state.baro_altitude = baro_pressure_to_altitude(state.baro_pressure);
  return true;
}
#else
baro_types_t baro_init(void) { return BARO_TYPE_INVALID; }
bool baro_update(void) { return false; }
#endif
