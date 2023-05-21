#include "core/target.h"

#include <string.h>

#include "driver/gpio.h"
#include "io/quic.h"
#include "util/cbor_helper.h"

target_t target = {
    .name = "unknown",
};

#define _MACRO_STR(arg) #arg
#define MACRO_STR(name) _MACRO_STR(name)

target_info_t target_info = {
    .mcu = MACRO_STR(TARGET_MCU),
    .git_version = MACRO_STR(GIT_VERSION),
    .quic_protocol_version = QUIC_PROTOCOL_VERSION,

#ifdef DEBUG
    .features = FEATURE_DEBUG,
#else
    .features = 0,
#endif
    .rx_protocols = {
        RX_PROTOCOL_UNIFIED_SERIAL,
    },

    .gyro_id = 0x0,
};

#define MEMBER CBOR_ENCODE_MEMBER
#define STR_MEMBER CBOR_ENCODE_STR_MEMBER
#define TSTR_MEMBER CBOR_ENCODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_ENCODE_ARRAY_MEMBER
#define INDEX_ARRAY_MEMBER CBOR_ENCODE_INDEX_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_ENCODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_ENCODER(target_led_t)
TARGET_LED_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_invert_pin_t)
TARGET_BUZZER_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_serial_port_t)
TARGET_SERIAL_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_spi_port_t)
TARGET_SPI_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_spi_device_t)
TARGET_SPI_DEVICE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_rx_spi_device_t)
TARGET_RX_SPI_DEVICE_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_t)
TARGET_MEMBERS
CBOR_END_STRUCT_ENCODER()

CBOR_START_STRUCT_ENCODER(target_info_t)
TARGET_INFO_MEMBERS
CBOR_END_STRUCT_ENCODER()

#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
#undef ARRAY_MEMBER
#undef INDEX_ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

#define MEMBER CBOR_DECODE_MEMBER
#define STR_MEMBER CBOR_DECODE_STR_MEMBER
#define TSTR_MEMBER CBOR_DECODE_TSTR_MEMBER
#define ARRAY_MEMBER CBOR_DECODE_ARRAY_MEMBER
#define INDEX_ARRAY_MEMBER CBOR_DECODE_INDEX_ARRAY_MEMBER
#define STR_ARRAY_MEMBER CBOR_DECODE_STR_ARRAY_MEMBER

CBOR_START_STRUCT_DECODER(target_led_t)
TARGET_LED_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_invert_pin_t)
TARGET_BUZZER_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_serial_port_t)
TARGET_SERIAL_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_spi_port_t)
TARGET_SPI_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_spi_device_t)
TARGET_SPI_DEVICE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_rx_spi_device_t)
TARGET_RX_SPI_DEVICE_MEMBERS
CBOR_END_STRUCT_DECODER()

CBOR_START_STRUCT_DECODER(target_t)
TARGET_MEMBERS
CBOR_END_STRUCT_DECODER()

#undef MEMBER
#undef STR_MEMBER
#undef TSTR_MEMBER
#undef ARRAY_MEMBER
#undef INDEX_ARRAY_MEMBER
#undef STR_ARRAY_MEMBER

static const uint8_t pin_none_str[] = "NONE";

typedef enum {
  GPIO_PORTA,
  GPIO_PORTB,
  GPIO_PORTC,
  GPIO_PORTD,
  GPIO_PORTE,
  GPIO_PORTF,
  GPIO_PORTG,
  GPIO_PORTH,
  GPIO_PORTI,
  GPIO_PORTJ,
  GPIO_PORTK,
  GPIO_PORTL,
  GPIO_PORTM,
  GPIO_PORTN,
  GPIO_PORTO,
  GPIO_PORTP,
  GPIO_PORT_MAX,
} gpio_ports_t;

#define GPIO_PIN_MAX 16

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(_port, _num) \
  [GPIO_PORT##_port * GPIO_PIN_MAX + _num] = PIN_##_port##_num,

static const gpio_pins_t gpio_pin_lookup[GPIO_PORT_MAX * GPIO_PIN_MAX] = {
#include "gpio_pins.in"
};

#undef GPIO_PIN
#undef GPIO_AF

#define GPIO_AF(pin, af, tag)
#define GPIO_PIN(_port, _num) \
  [PIN_##_port##_num] = GPIO_PORT##_port,

static const gpio_ports_t gpio_port_lookup[PINS_MAX] = {
#include "gpio_pins.in"
};

#undef GPIO_PIN
#undef GPIO_AF

cbor_result_t cbor_encode_gpio_pins_t(cbor_value_t *enc, const gpio_pins_t *t) {
  cbor_result_t res = CBOR_OK;

  uint8_t buf[4];
  uint8_t size = 3;

  if (*t != PIN_NONE) {
    const uint8_t port = gpio_port_lookup[*t];
    const uint8_t pin = gpio_pin_defs[*t].pin_index;

    buf[0] = 'P';
    buf[1] = 'A' + port;

    if (pin >= 10) {
      buf[2] = '1';
      buf[3] = '0' + pin % 10;
      size = 4;
    } else {
      buf[2] = '0' + pin;
    }
  } else {
    memcpy(buf, pin_none_str, 4);
    size = 4;
  }

  CBOR_CHECK_ERROR(res = cbor_encode_tstr(enc, buf, size));

  return res;
}

cbor_result_t cbor_decode_gpio_pins_t(cbor_value_t *dec, gpio_pins_t *t) {
  cbor_result_t res = CBOR_OK;

  const uint8_t *buf;
  uint32_t size;
  CBOR_CHECK_ERROR(res = cbor_decode_tstr(dec, &buf, &size));

  if (size > 4 && size < 3) {
    return CBOR_ERR_INVALID_TYPE;
  }

  if (size == 4 && memcmp(buf, pin_none_str, 4) == 0) {
    *t = PIN_NONE;
    return res;
  }

  if (buf[0] != 'P') {
    return CBOR_ERR_INVALID_TYPE;
  }

  uint32_t val = (buf[1] - 'A') * GPIO_PORT_MAX;
  if (size == 4) {
    val += 10;
    val += buf[3] - '0';
  } else {
    val += buf[2] - '0';
  }

  *t = gpio_pin_lookup[val];

  return res;
}

void target_set_feature(target_feature_t feat) {
  target_info.features |= feat;
}

void target_reset_feature(target_feature_t feat) {
  target_info.features &= ~feat;
}

void target_add_rx_protocol(rx_protocol_t proto) {
  for (uint32_t i = 0; i < RX_PROTOCOL_MAX; i++) {
    if (target_info.rx_protocols[i] == proto) {
      // we already have proto
      break;
    }
    if (target_info.rx_protocols[i] == RX_PROTOCOL_INVALID) {
      // add proto
      target_info.rx_protocols[i] = proto;
      break;
    }
  }
}

bool target_has_rx_protocol(rx_protocol_t proto) {
  for (uint32_t i = 0; i < RX_PROTOCOL_MAX; i++) {
    if (target_info.rx_protocols[i] == proto) {
      // we already have proto
      return true;
    }
    if (target_info.rx_protocols[i] == RX_PROTOCOL_INVALID) {
      return false;
    }
  }
  return false;
}

bool target_serial_port_valid(const target_serial_port_t *port) {
  return port->index != 0 && (port->rx != PIN_NONE || port->tx != PIN_NONE);
}

bool target_spi_device_valid(const target_spi_device_t *dev) {
  return dev->port != SPI_PORT_INVALID && dev->nss != PIN_NONE;
}

bool target_spi_port_valid(const target_spi_port_t *port) {
  return port->index != 0 && port->miso != PIN_NONE && port->mosi != PIN_NONE && port->sck != PIN_NONE;
}
