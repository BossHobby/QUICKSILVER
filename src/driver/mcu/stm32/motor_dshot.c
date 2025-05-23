#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"

#ifdef USE_MOTOR_DSHOT

static void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;
  LL_TIM_OC_Init(TIM1, timer_channel_val(port->timer_channel), &tim_oc_init);
  LL_TIM_OC_EnablePreload(TIM1, timer_channel_val(port->timer_channel));

  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  dma_enable_rcc(port->dma_device);
  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = dma->request;
#else
  DMA_InitStructure.Channel = dma->channel;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = 0x0; // overwritten later
  DMA_InitStructure.MemoryOrM2MDstAddress = 0x0; // overwritten later
  DMA_InitStructure.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
  DMA_InitStructure.NbData = DSHOT_DMA_BUFFER_SIZE;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
#ifndef STM32G4
  DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
#endif
  LL_DMA_Init(dma->port, dma->stream_index, &DMA_InitStructure);
  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);

  interrupt_enable(dma->irq, DMA_PRIORITY);
}

void dshot_init_timer() {
  rcc_enable(RCC_APB2_GRP1(TIM1));

  // setup timer to 1/3 of the full bit time
  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(TIM1, &tim_init);
  LL_TIM_EnableARRPreload(TIM1);
  LL_TIM_DisableMasterSlaveMode(TIM1);

  for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
    dshot_init_gpio_port(&dshot_gpio_ports[j]);

    for (uint8_t i = 0; i < DSHOT_DMA_SYMBOLS; i++) {
      dshot_output_buffer[j][i * 3 + 0] = dshot_gpio_ports[j].set_mask;   // start bit
      dshot_output_buffer[j][i * 3 + 1] = 0;                              // actual bit, set below
      dshot_output_buffer[j][i * 3 + 2] = dshot_gpio_ports[j].reset_mask; // return line to low
    }
  }

  LL_TIM_EnableCounter(TIM1);
}

void dshot_dma_setup_output(uint32_t index) {
  LL_TIM_SetAutoReload(TIM1, DSHOT_SYMBOL_TIME);

  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&port->gpio->BSRR);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&dshot_output_buffer[index][0]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, DSHOT_DMA_BUFFER_SIZE);
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  timer_enable_dma_request(TIMER1, port->timer_channel, true);
}

void dshot_dma_setup_input(uint32_t index) {
  LL_TIM_SetAutoReload(TIM1, GCR_SYMBOL_TIME);

  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_stream_def_t *dma = &dma_stream_defs[port->dma_device];

  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&port->gpio->IDR);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&dshot_input_buffer[index][0]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, GCR_DMA_BUFFER_SIZE);
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  timer_enable_dma_request(TIMER1, port->timer_channel, true);
}

void dshot_dma_isr(dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[dev];
  dma_clear_flag_tc(dma);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  timer_enable_dma_request(TIMER1, port->timer_channel, false);

  dshot_phase--;

  if (profile.motor.dshot_telemetry && dshot_phase == dshot_gpio_port_count) {
    // output phase done, lets swap to input
    for (uint32_t i = 0; i < MOTOR_PIN_MAX; i++) {
      dshot_gpio_init_input(target.motor_pins[i]);
    }
    for (uint32_t j = 0; j < dshot_gpio_port_count; j++) {
      dshot_dma_setup_input(j);
    }
  }
}
#endif