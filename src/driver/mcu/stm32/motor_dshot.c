#include "driver/motor_dshot.h"

#include <stdint.h>

#include "core/failloop.h"
#include "core/profile.h"
#include "core/project.h"
#include "driver/dma.h"
#include "driver/gpio.h"
#include "driver/interrupt.h"
#include "driver/rcc.h"

#ifdef USE_MOTOR_DSHOT

void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];

  rcc_enable(tim->rcc);

  // setup timer to 1/3 of the full bit time
  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(tim->instance, &tim_init);
  LL_TIM_EnableARRPreload(tim->instance);
  LL_TIM_DisableMasterSlaveMode(tim->instance);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;

  const uint32_t ch = timer_channel_val(TIMER_TAG_CH(port->timer_tag));
  LL_TIM_OC_Init(tim->instance, ch, &tim_oc_init);
  LL_TIM_OC_EnablePreload(tim->instance, ch);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  dma_enable_rcc(dma);
  LL_DMA_DeInit(dma->port, dma->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = target.dma[port->dma_device].request;
#else
  DMA_InitStructure.Channel = dma_map_channel(target.dma[port->dma_device].channel);
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

  interrupt_enable(dma->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma->port, dma->stream_index);

  LL_TIM_EnableCounter(tim->instance);
}

void dshot_dma_setup_output(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];

  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];
  LL_TIM_SetAutoReload(tim->instance, DSHOT_SYMBOL_TIME);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&port->gpio->BSRR);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&dshot_output_buffer[index][0]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, DSHOT_DMA_BUFFER_SIZE);
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_WORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_WORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void dshot_dma_setup_input(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];

  const timer_def_t *tim = &timer_defs[TIMER_TAG_TIM(port->timer_tag)];
  LL_TIM_SetAutoReload(tim->instance, GCR_SYMBOL_TIME);

  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[port->dma_device].dma];
  LL_DMA_SetPeriphAddress(dma->port, dma->stream_index, (uint32_t)&port->gpio->IDR);
  LL_DMA_SetMemoryAddress(dma->port, dma->stream_index, (uint32_t)&dshot_input_buffer[index][0]);
  LL_DMA_SetDataLength(dma->port, dma->stream_index, GCR_DMA_BUFFER_SIZE);
  LL_DMA_SetDataTransferDirection(dma->port, dma->stream_index, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  LL_DMA_SetPeriphSize(dma->port, dma->stream_index, LL_DMA_PDATAALIGN_HALFWORD);
  LL_DMA_SetMemorySize(dma->port, dma->stream_index, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_EnableStream(dma->port, dma->stream_index);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void dshot_dma_isr(const dma_device_t dev) {
  const dma_stream_def_t *dma = &dma_stream_defs[target.dma[dev].dma];

  dma_clear_flag_tc(dma);
  LL_DMA_DisableStream(dma->port, dma->stream_index);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(dev);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), false);

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