#include "driver/adc.h"

// hardware/adc.h would clash with the driver API's adc_init().
#include <hardware/regs/adc.h>
#include <hardware/structs/adc.h>

#include "driver/interrupt.h"

#ifdef USE_ADC

// Free-running round robin over the configured inputs at about 1 kHz in
// total; the ADC samples once every (1 + div) cycles of its 48 MHz clock.
static constexpr uint32_t ADC_CLKDIV = 47999;

// Temperature sensor: 0.706 V at 27 degC, -1.721 mV/degC (RP2350 datasheet).
static constexpr float ADC_TEMP_V27 = 0.706f;
static constexpr float ADC_TEMP_SLOPE = -0.001721f;

static constexpr uint32_t ADC_TEMPERATURE_CHANNEL_NUM = NUM_ADC_CHANNELS - 1;

// Configured channels in input order, the order round robin visits them.
static adc_chan_t adc_sequence[ADC_CHAN_MAX];
static uint32_t adc_sequence_len;
static uint32_t adc_sequence_pos;

static void adc_init_pin(adc_chan_t chan, gpio_pins_t pin) {
  adc_pins[chan].pin = PIN_NONE;
  adc_pins[chan].dev = ADC_DEVICE_MAX;

  switch (chan) {
  case ADC_CHAN_VREF:
    // No internal reference; see ADC_SUPPLY_REF_MV.
    break;

  case ADC_CHAN_TEMP:
    adc_pins[chan].dev = ADC_DEVICE1;
    adc_pins[chan].channel = ADC_TEMPERATURE_CHANNEL_NUM;
    break;

  default:
    if (pin == PIN_NONE)
      break;
    for (uint32_t i = 0; i < GPIO_AF_MAX; i++) {
      const gpio_af_t *func = &gpio_pin_afs[i];
      if (func->pin != pin || RESOURCE_TAG_TYPE(func->tag) != RESOURCE_ADC) {
        continue;
      }

      adc_pins[chan].pin = pin;
      adc_pins[chan].dev = ADC_TAG_DEV(func->tag);
      adc_pins[chan].channel = ADC_TAG_CH(func->tag);
      break;
    }
    break;
  }

  if (adc_pins[chan].pin != PIN_NONE) {
    gpio_config_t gpio_init = gpio_config_default();
    gpio_init.mode = GPIO_ANALOG;
    gpio_pin_init(pin, gpio_init);
  }
}

static void adc_sequence_restart() {
  hw_clear_bits(&adc_hw->cs, ADC_CS_START_MANY_BITS);
  while (!(adc_hw->cs & ADC_CS_READY_BITS))
    ;
  while (!(adc_hw->fcs & ADC_FCS_EMPTY_BITS))
    (void)adc_hw->fifo;
  hw_write_masked(&adc_hw->cs, adc_pins[adc_sequence[0]].channel << ADC_CS_AINSEL_LSB, ADC_CS_AINSEL_BITS);
  adc_sequence_pos = 0;
  hw_set_bits(&adc_hw->cs, ADC_CS_START_MANY_BITS);
}

void adc_init_hardware() {
  for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
    const auto chan = static_cast<adc_chan_t>(i);
    adc_init_pin(chan, chan == ADC_CHAN_VBAT ? target.vbat : chan == ADC_CHAN_IBAT ? target.ibat
                                                                                   : PIN_NONE);
  }

  uint32_t mask = 0;
  adc_sequence_len = 0;
  for (uint32_t input = 0; input < NUM_ADC_CHANNELS; input++) {
    for (uint32_t i = 0; i < ADC_CHAN_MAX; i++) {
      if (adc_pins[i].dev == ADC_DEVICE_MAX || adc_pins[i].channel != input)
        continue;
      adc_sequence[adc_sequence_len++] = static_cast<adc_chan_t>(i);
      mask |= 1u << input;
    }
  }

  reset_unreset_block_num_wait_blocking(RESET_ADC);
  adc_hw->cs = ADC_CS_EN_BITS | ADC_CS_TS_EN_BITS | (mask << ADC_CS_RROBIN_LSB);
  adc_hw->div = (uint32_t)ADC_CLKDIV << ADC_DIV_INT_LSB;
  // Interrupt on every result.
  adc_hw->fcs = ADC_FCS_EN_BITS | (1u << ADC_FCS_THRESH_LSB);
  adc_hw->inte = ADC_INTE_FIFO_BITS;
  interrupt_enable(ADC_IRQ_FIFO_IRQn, ADC_PRIORITY);

  adc_sequence_restart();
}

extern "C" void ADC_IRQ_FIFO_Handler() {
  // A lost sample would shift every later result onto the wrong channel.
  if (adc_hw->fcs & ADC_FCS_OVER_BITS) {
    hw_set_bits(&adc_hw->fcs, ADC_FCS_OVER_BITS);
    adc_sequence_restart();
    return;
  }

  while (!(adc_hw->fcs & ADC_FCS_EMPTY_BITS)) {
    adc_accumulate(adc_sequence[adc_sequence_pos], adc_hw->fifo & ADC_FIFO_VAL_BITS);
    adc_sequence_pos = (adc_sequence_pos + 1) % adc_sequence_len;
  }
}

float adc_convert_to_temp(float val) {
  const float volts = val * (ADC_SUPPLY_REF_MV / 1000.0f) / 4095.0f;
  return (volts - ADC_TEMP_V27) / ADC_TEMP_SLOPE + 27.0f;
}

#endif
