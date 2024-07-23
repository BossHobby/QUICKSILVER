#include "driver/i2c.h"

// Clock period in us during unstick transfer
#define UNSTICK_CLK_US 10 // 100Khz
// Allow 500us for clock stretch to complete during unstick
#define UNSTICK_CLK_STRETCH (500 / UNSTICK_CLK_US)

FAST_RAM i2c_device_t i2c_dev[I2C_PORT_MAX] = {
    [RANGE_INIT(0, I2C_PORT_MAX)] = {.is_init = false},
};

extern void i2c_device_init(i2c_ports_t port);

static void i2c_init_pins(i2c_ports_t port) {
  const target_i2c_port_t *dev = &target.i2c_ports[port];

  gpio_config_t gpio_init;

  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_OPENDRAIN;
  gpio_init.pull = GPIO_UP_PULL;
  gpio_pin_init_tag(dev->sda, gpio_init, I2C_TAG(port, RES_I2C_SDA));

  gpio_init.mode = GPIO_ALTERNATE;
  gpio_init.drive = GPIO_DRIVE_HIGH;
  gpio_init.output = GPIO_OPENDRAIN;
  gpio_init.pull = GPIO_UP_PULL;
  gpio_pin_init_tag(dev->scl, gpio_init, I2C_TAG(port, RES_I2C_SCL));
}

static bool i2c_unstick_wait_stretch(gpio_pins_t scl, uint32_t timeout) {
  for (uint32_t i = 0; i < timeout; i++) {
    if (gpio_pin_read(scl))
      return true;
    time_delay_us(UNSTICK_CLK_US);
  }
  return false;
}

static bool i2c_unstick(i2c_ports_t port) {
  const target_i2c_port_t *dev = &target.i2c_ports[port];

  gpio_pin_set(dev->sda);
  gpio_pin_set(dev->scl);

  gpio_config_t gpio_init = {
      .mode = GPIO_OUTPUT,
      .drive = GPIO_DRIVE_HIGH,
      .output = GPIO_OPENDRAIN,
      .pull = GPIO_NO_PULL,
  };
  gpio_pin_init_tag(dev->sda, gpio_init, I2C_TAG(port, RES_I2C_SDA));
  gpio_pin_init_tag(dev->scl, gpio_init, I2C_TAG(port, RES_I2C_SCL));

  // Clock out, with SDA high:
  // 7 data bits, 1 READ bit, 1 cycle for the ACK
  for (uint32_t i = 0; i < (7 + 1 + 1); i++) {
    // Wait for any clock stretching to finish
    i2c_unstick_wait_stretch(dev->scl, UNSTICK_CLK_STRETCH);
    // Pull low
    gpio_pin_reset(dev->scl); // Set bus low
    time_delay_us(UNSTICK_CLK_US / 2);
    gpio_pin_set(dev->scl); // Set bus high
    time_delay_us(UNSTICK_CLK_US / 2);
  }

  // slave may be still stretching after last pulse
  i2c_unstick_wait_stretch(dev->scl, UNSTICK_CLK_STRETCH);

  // Generate a stop condition in case there was none
  // SCL low pulse to switch SDA low
  gpio_pin_reset(dev->scl);
  time_delay_us(UNSTICK_CLK_US / 2);
  gpio_pin_reset(dev->sda);
  time_delay_us(UNSTICK_CLK_US / 2);
  gpio_pin_set(dev->scl);
  time_delay_us(UNSTICK_CLK_US / 2);

  // SDA rising edge = STOP
  gpio_pin_set(dev->sda);

  // check that both SCL and SDA are high
  time_delay_us(UNSTICK_CLK_US / 2); // time for SDA to return high

  return gpio_pin_read(dev->scl) && gpio_pin_read(dev->sda);
}

bool i2c_bus_device_init(const i2c_bus_device_t *bus) {
  if (!target_i2c_port_valid(&target.i2c_ports[bus->port])) {
    return false;
  }

  if (!i2c_dev[bus->port].is_init) {
    i2c_unstick(bus->port);
    time_delay_ms(10);
    i2c_init_pins(bus->port);
    i2c_device_init(bus->port);
    i2c_dev[bus->port].is_init = true;
  }

  return true;
}

uint32_t i2c_calc_clkctrl(uint32_t pclk_freq, uint32_t i2c_freq_khz, uint32_t dfcoeff) {
  // Silicon specific values, from datasheet
  const uint8_t tAFmin = 50; // Analog filter delay (min)

  // Actual (estimated) values
  const uint8_t tr = 100; // Rise time
  const uint8_t tf = 10;  // Fall time

  // Values from I2C-SMBus specification
  const uint16_t trmax = i2c_freq_khz > 400 ? 120 : 300;    // Rise time (max)
  const uint16_t tfmax = i2c_freq_khz > 400 ? 120 : 300;    // Fall time (max)
  const uint8_t tsuDATmin = i2c_freq_khz > 400 ? 50 : 100;  // SDA setup time (min)
  const uint8_t thdDATmin = i2c_freq_khz > 400 ? 0 : 0;     // SDA hold time (min)
  const uint16_t tHIGHmin = i2c_freq_khz > 400 ? 260 : 600; // High period of SCL clock (min)
  const uint16_t tLOWmin = i2c_freq_khz > 400 ? 500 : 1300; // Low period of SCL clock (min)

  // Convert pclk_freq into nsec
  const float tI2cclk = 1000000000.0f / pclk_freq;

  // Convert target i2cFreq into cycle time (nsec)
  const float tSCL = 1000000.0f / i2c_freq_khz;

  for (uint32_t presc = 0; presc < 15; presc++) {
    const uint8_t scldel = (trmax + tsuDATmin) / ((presc + 1) * tI2cclk) - 1;
    const uint8_t sdadel = (tfmax + thdDATmin - tAFmin - ((dfcoeff + 3) * tI2cclk)) / ((presc + 1) * tI2cclk);

    const float tsync1 = tf + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;
    const float tsync2 = tr + tAFmin + dfcoeff * tI2cclk + 2 * tI2cclk;

    const float tSCLH = tHIGHmin * tSCL / (tHIGHmin + tLOWmin) - tsync2;
    const float tSCLL = tSCL - tSCLH - tsync1 - tsync2;

    uint16_t sclh = tSCLH / ((presc + 1) * tI2cclk) - 1;
    uint16_t scll = tSCLL / ((presc + 1) * tI2cclk) - 1;

    while (tsync1 + tsync2 + ((sclh + 1) + (scll + 1)) * ((presc + 1) * tI2cclk) < tSCL) {
      sclh++;
    }

    // If all fields are not overflowing, return TIMINGR.
    // Otherwise, increase prescaler and try again.
    if ((scldel < 16) && (sdadel < 16) && (sclh < 256) && (scll < 256)) {
      return ((presc << 28) | (scldel << 20) | (sdadel << 16) | (sclh << 8) | (scll << 0));
    }
  }

  return 0; // Shouldn't reach here
}

void i2c_wait_idle(const i2c_bus_device_t *bus) {
  while (!i2c_is_idle(bus))
    ;
}

void i2c_write_reg(const i2c_bus_device_t *bus, const uint8_t reg, const uint8_t data) {
  i2c_write_reg_bytes(bus, reg, &data, 1);
  i2c_wait_idle(bus);
}

uint8_t i2c_read_reg(const i2c_bus_device_t *bus, const uint8_t reg) {
  uint8_t data = 0;
  i2c_read_reg_bytes(bus, reg, &data, 1);
  i2c_wait_idle(bus);
  return data;
}