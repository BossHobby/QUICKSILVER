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

extern uint16_t dshot_packet[MOTOR_PIN_MAX];
extern dshot_pin_t dshot_pins[MOTOR_PIN_MAX];
extern volatile uint32_t dshot_dma_phase;

extern uint8_t dshot_gpio_port_count;
extern dshot_gpio_port_t dshot_gpio_ports[DSHOT_MAX_PORT_COUNT];

extern volatile DMA_RAM uint32_t port_dma_buffer[DSHOT_MAX_PORT_COUNT][DSHOT_DMA_BUFFER_SIZE];

static const resource_tag_t timers[] = {
#ifndef STM32F411
    TIMER_TAG(TIMER8, TIMER_CH1),
    TIMER_TAG(TIMER8, TIMER_CH2),
    TIMER_TAG(TIMER8, TIMER_CH3),
    TIMER_TAG(TIMER8, TIMER_CH4),
#endif
    TIMER_TAG(TIMER1, TIMER_CH1),
    TIMER_TAG(TIMER1, TIMER_CH2),
    TIMER_TAG(TIMER1, TIMER_CH3),
    TIMER_TAG(TIMER1, TIMER_CH4),
};
static const uint32_t timer_count = sizeof(timers) / sizeof(resource_tag_t);

extern const dshot_gpio_port_t *dshot_gpio_for_device(const dma_device_t dev);

void dshot_init_gpio_port(dshot_gpio_port_t *port) {
  for (uint8_t i = 0; i < timer_count; i++) {
    const resource_tag_t tag = timers[i];
    if (timer_alloc_tag(TIMER_USE_MOTOR_DSHOT, tag)) {
      port->timer_tag = tag;
      break;
    }
  }
  if (port->timer_tag == 0) {
    failloop(FAILLOOP_DMA);
  }

  const dma_assigment_t *dma = port->dma_ass = dma_alloc(port->dma_device, port->timer_tag);
  if (dma == NULL) {
    failloop(FAILLOOP_DMA);
  }

  const timer_channel_t ch = TIMER_TAG_CH(port->timer_tag);
  const timer_index_t tim = TIMER_TAG_TIM(port->timer_tag);
  const timer_def_t *def = &timer_defs[tim];

  rcc_enable(def->rcc);

  // setup timer to 1/3 of the full bit time
  LL_TIM_InitTypeDef tim_init;
  LL_TIM_StructInit(&tim_init);
  tim_init.Autoreload = DSHOT_SYMBOL_TIME;
  tim_init.Prescaler = 0;
  tim_init.ClockDivision = 0;
  tim_init.CounterMode = LL_TIM_COUNTERMODE_UP;
  LL_TIM_Init(def->instance, &tim_init);
  LL_TIM_EnableARRPreload(def->instance);
  LL_TIM_DisableMasterSlaveMode(def->instance);

  LL_TIM_OC_InitTypeDef tim_oc_init;
  LL_TIM_OC_StructInit(&tim_oc_init);
  tim_oc_init.OCMode = LL_TIM_OCMODE_PWM1;
  tim_oc_init.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
  tim_oc_init.OCState = LL_TIM_OCSTATE_ENABLE;
  tim_oc_init.OCPolarity = LL_TIM_OCPOLARITY_LOW;
  tim_oc_init.CompareValue = 10;
  LL_TIM_OC_Init(def->instance, ch, &tim_oc_init);
  LL_TIM_OC_EnablePreload(def->instance, ch);

  dma_enable_rcc(dma);
  LL_DMA_DeInit(dma->def->port, dma->def->stream_index);

  LL_DMA_InitTypeDef DMA_InitStructure;
  LL_DMA_StructInit(&DMA_InitStructure);
#if defined(STM32H7) || defined(STM32G4)
  DMA_InitStructure.PeriphRequest = dma->chan->request;
#else
  DMA_InitStructure.Channel = dma->chan->channel;
#endif
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&port->gpio->BSRR;
  DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)port_dma_buffer[0];
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
  LL_DMA_Init(dma->def->port, dma->def->stream_index, &DMA_InitStructure);

  interrupt_enable(dma->def->irq, DMA_PRIORITY);
  LL_DMA_EnableIT_TC(dma->def->port, dma->def->stream_index);

  LL_TIM_EnableCounter(def->instance);
}

void dshot_dma_setup_port(uint32_t index) {
  const dshot_gpio_port_t *port = &dshot_gpio_ports[index];
  const dma_assigment_t *dma = port->dma_ass;

  dma_clear_flag_tc(dma->def);

  LL_DMA_SetPeriphAddress(dma->def->port, dma->def->stream_index, (uint32_t)&port->gpio->BSRR);
  LL_DMA_SetMemoryAddress(dma->def->port, dma->def->stream_index, (uint32_t)&port_dma_buffer[index][0]);
  LL_DMA_SetDataLength(dma->def->port, dma->def->stream_index, DSHOT_DMA_BUFFER_SIZE);

  LL_DMA_EnableStream(dma->def->port, dma->def->stream_index);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), true);
}

void motor_dshot_wait_for_ready() {
  while (dshot_dma_phase != 0)
    __NOP();
}

void dshot_dma_isr(const dma_assigment_t *ass) {
  dma_clear_flag_tc(ass->def);
  LL_DMA_DisableStream(ass->def->port, ass->def->stream_index);

  const dshot_gpio_port_t *port = dshot_gpio_for_device(ass->dev);
  timer_enable_dma_request(TIMER_TAG_TIM(port->timer_tag), TIMER_TAG_CH(port->timer_tag), false);

  dshot_dma_phase--;
}
#endif