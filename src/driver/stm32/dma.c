#include "driver/dma.h"

#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#define DMA_STREAM_MAX 16

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

#define DMA_STREAM(_port, _stream)              \
  {                                             \
    .port = DMA##_port,                         \
    .port_index = _port,                        \
    .stream = DMA##_port##_Stream##_stream,     \
    .stream_index = LL_DMA_STREAM_##_stream,    \
    .irq = DMA##_port##_Stream##_stream##_IRQn, \
  }
static const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {
    DMA_STREAM(1, 0),
    DMA_STREAM(1, 1),
    DMA_STREAM(1, 2),
    DMA_STREAM(1, 3),
    DMA_STREAM(1, 4),
    DMA_STREAM(1, 5),
    DMA_STREAM(1, 6),
    DMA_STREAM(1, 7),
    DMA_STREAM(2, 0),
    DMA_STREAM(2, 1),
    DMA_STREAM(2, 2),
    DMA_STREAM(2, 3),
    DMA_STREAM(2, 4),
    DMA_STREAM(2, 5),
    DMA_STREAM(2, 6),
    DMA_STREAM(2, 7),
};
#undef DMA_STREAM

typedef struct {
  resource_tag_t tag;
  uint8_t port_index;
  uint8_t stream_index;
  uint32_t channel;
} dma_channel_t;

static const dma_channel_t dma_channels[] = {
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MISO), .port_index = 2, .stream_index = 0, .channel = LL_DMA_CHANNEL_3},
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MISO), .port_index = 2, .stream_index = 2, .channel = LL_DMA_CHANNEL_3},
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MOSI), .port_index = 2, .stream_index = 3, .channel = LL_DMA_CHANNEL_3},
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MOSI), .port_index = 2, .stream_index = 5, .channel = LL_DMA_CHANNEL_3},
    {.tag = SPI_TAG(SPI_PORT2, RES_SPI_MISO), .port_index = 1, .stream_index = 3, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT2, RES_SPI_MOSI), .port_index = 1, .stream_index = 4, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MISO), .port_index = 1, .stream_index = 0, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MISO), .port_index = 1, .stream_index = 3, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MOSI), .port_index = 1, .stream_index = 5, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MOSI), .port_index = 1, .stream_index = 7, .channel = LL_DMA_CHANNEL_0},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MISO), .port_index = 2, .stream_index = 0, .channel = LL_DMA_CHANNEL_4},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MISO), .port_index = 2, .stream_index = 3, .channel = LL_DMA_CHANNEL_5},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MOSI), .port_index = 2, .stream_index = 1, .channel = LL_DMA_CHANNEL_4},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MOSI), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER1, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER2, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER3, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER4, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER5, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},

    {.tag = TIMER_TAG(TIMER8, TIMER_CH1), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH2), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH3), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH4), .port_index = 2, .stream_index = 4, .channel = LL_DMA_CHANNEL_5},
};

#define DMA_CHANNEL_MAX (sizeof(dma_channels) / sizeof(dma_channel_t))

static dma_assigment_t dma_assigments[DMA_STREAM_MAX] = {};

void dma_prepare_tx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_prepare_rx_memory(void *addr, uint32_t size) {
#if defined(STM32F7) || defined(STM32H7)
  if (!WITHIN_DTCM_RAM(addr) && !WITHIN_DMA_RAM(addr)) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)((uint32_t)addr & ~CACHE_LINE_MASK), (size + CACHE_LINE_SIZE) & ~CACHE_LINE_MASK);
  }
#endif
}

void dma_enable_rcc(const dma_assigment_t *ass) {
  switch (ass->def->port_index) {
  case 1:
    rcc_enable(RCC_AHB1_GRP1(DMA1));
    break;
  case 2:
    rcc_enable(RCC_AHB1_GRP1(DMA2));
    break;
  }
}

bool dma_is_flag_active_tc(const dma_assigment_t *ass) {
  switch (ass->def->stream_index) {
  case LL_DMA_STREAM_0:
    return LL_DMA_IsActiveFlag_TC0(ass->def->port);
  case LL_DMA_STREAM_1:
    return LL_DMA_IsActiveFlag_TC1(ass->def->port);
  case LL_DMA_STREAM_2:
    return LL_DMA_IsActiveFlag_TC2(ass->def->port);
  case LL_DMA_STREAM_3:
    return LL_DMA_IsActiveFlag_TC3(ass->def->port);
  case LL_DMA_STREAM_4:
    return LL_DMA_IsActiveFlag_TC4(ass->def->port);
  case LL_DMA_STREAM_5:
    return LL_DMA_IsActiveFlag_TC5(ass->def->port);
  case LL_DMA_STREAM_6:
    return LL_DMA_IsActiveFlag_TC6(ass->def->port);
  case LL_DMA_STREAM_7:
    return LL_DMA_IsActiveFlag_TC7(ass->def->port);
  }
  return 0;
}

void dma_clear_flag_tc(const dma_assigment_t *ass) {
  switch (ass->def->stream_index) {
  case LL_DMA_STREAM_0:
    LL_DMA_ClearFlag_TC0(ass->def->port);
    LL_DMA_ClearFlag_TE0(ass->def->port);
    LL_DMA_ClearFlag_HT0(ass->def->port);
    LL_DMA_ClearFlag_FE0(ass->def->port);
    break;
  case LL_DMA_STREAM_1:
    LL_DMA_ClearFlag_TC1(ass->def->port);
    LL_DMA_ClearFlag_TE1(ass->def->port);
    LL_DMA_ClearFlag_HT1(ass->def->port);
    LL_DMA_ClearFlag_FE1(ass->def->port);
    break;
  case LL_DMA_STREAM_2:
    LL_DMA_ClearFlag_TC2(ass->def->port);
    LL_DMA_ClearFlag_TE2(ass->def->port);
    LL_DMA_ClearFlag_HT2(ass->def->port);
    LL_DMA_ClearFlag_FE2(ass->def->port);
    break;
  case LL_DMA_STREAM_3:
    LL_DMA_ClearFlag_TC3(ass->def->port);
    LL_DMA_ClearFlag_TE3(ass->def->port);
    LL_DMA_ClearFlag_HT3(ass->def->port);
    LL_DMA_ClearFlag_FE3(ass->def->port);
    break;
  case LL_DMA_STREAM_4:
    LL_DMA_ClearFlag_TC4(ass->def->port);
    LL_DMA_ClearFlag_TE4(ass->def->port);
    LL_DMA_ClearFlag_HT4(ass->def->port);
    LL_DMA_ClearFlag_FE4(ass->def->port);
    break;
  case LL_DMA_STREAM_5:
    LL_DMA_ClearFlag_TC5(ass->def->port);
    LL_DMA_ClearFlag_TE5(ass->def->port);
    LL_DMA_ClearFlag_HT5(ass->def->port);
    LL_DMA_ClearFlag_FE5(ass->def->port);
    break;
  case LL_DMA_STREAM_6:
    LL_DMA_ClearFlag_TC6(ass->def->port);
    LL_DMA_ClearFlag_TE6(ass->def->port);
    LL_DMA_ClearFlag_HT6(ass->def->port);
    LL_DMA_ClearFlag_FE6(ass->def->port);
    break;
  case LL_DMA_STREAM_7:
    LL_DMA_ClearFlag_TC7(ass->def->port);
    LL_DMA_ClearFlag_TE7(ass->def->port);
    LL_DMA_ClearFlag_HT7(ass->def->port);
    LL_DMA_ClearFlag_FE7(ass->def->port);
    break;
  }
}

extern void dshot_dma_isr(const dma_assigment_t *);
extern void spi_dma_isr(const dma_assigment_t *);
extern void rgb_dma_isr(const dma_assigment_t *);

static void handle_dma_stream_isr(const dma_assigment_t *ass) {
  switch (ass->dev) {
  case DMA_DEVICE_TIM1_CH1:
  case DMA_DEVICE_TIM1_CH3:
  case DMA_DEVICE_TIM1_CH4:
#ifdef USE_MOTOR_DSHOT
    dshot_dma_isr(ass);
#endif
    break;

  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(ass);
    break;

  case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
    rgb_dma_isr(ass);
#endif
    break;

  case DMA_DEVICE_MAX:
    break;
  }
}

void DMA1_Stream0_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[0]);
}
void DMA1_Stream1_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[1]);
}
void DMA1_Stream2_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[2]);
}
void DMA1_Stream3_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[3]);
}
void DMA1_Stream4_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[4]);
}
void DMA1_Stream5_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[5]);
}
void DMA1_Stream6_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[6]);
}
void DMA1_Stream7_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[7]);
}

void DMA2_Stream0_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[8]);
}
void DMA2_Stream1_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[9]);
}
void DMA2_Stream2_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[10]);
}
void DMA2_Stream3_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[11]);
}
void DMA2_Stream4_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[12]);
}
void DMA2_Stream5_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[13]);
}
void DMA2_Stream6_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[14]);
}
void DMA2_Stream7_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[15]);
}