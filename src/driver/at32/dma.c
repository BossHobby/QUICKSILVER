#include "driver/dma.h"

#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#define DMA_CHANNEL_MAX 14

#define DMA_GL_FLAG ((uint32_t)0x00000001)
#define DMA_FDT_FLAG ((uint32_t)0x00000002)
#define DMA_HDT_FLAG ((uint32_t)0x00000004)
#define DMA_DTERR_FLAG ((uint32_t)0x00000008)

#define DMA_CHANNEL(_port, _chan)              \
  {                                            \
    .port = DMA##_port,                        \
    .port_index = _port,                       \
    .channel = DMA##_port##_CHANNEL##_chan,    \
    .channel_index = _chan,                    \
    .mux = DMA##_port##MUX_CHANNEL##_chan,     \
    .irq = DMA##_port##_Channel##_chan##_IRQn, \
  }

static const dma_channel_def_t dma_channel_defs[DMA_CHANNEL_MAX] = {
    DMA_CHANNEL(1, 1),
    DMA_CHANNEL(1, 2),
    DMA_CHANNEL(1, 3),
    DMA_CHANNEL(1, 4),
    DMA_CHANNEL(1, 5),
    DMA_CHANNEL(1, 6),
    DMA_CHANNEL(1, 7),
    DMA_CHANNEL(2, 1),
    DMA_CHANNEL(2, 2),
    DMA_CHANNEL(2, 3),
    DMA_CHANNEL(2, 4),
    DMA_CHANNEL(2, 5),
    DMA_CHANNEL(2, 6),
    DMA_CHANNEL(2, 7),
};

#undef DMA_CHANNEL

typedef struct {
  resource_tag_t tag;
  dmamux_requst_id_sel_type request;
} dma_req_id_t;

static const dma_req_id_t dma_req_ids[] = {
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MISO), .request = DMAMUX_DMAREQ_ID_SPI1_RX},
    {.tag = SPI_TAG(SPI_PORT1, RES_SPI_MOSI), .request = DMAMUX_DMAREQ_ID_SPI1_TX},
    {.tag = SPI_TAG(SPI_PORT2, RES_SPI_MISO), .request = DMAMUX_DMAREQ_ID_SPI2_RX},
    {.tag = SPI_TAG(SPI_PORT2, RES_SPI_MOSI), .request = DMAMUX_DMAREQ_ID_SPI2_TX},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MISO), .request = DMAMUX_DMAREQ_ID_SPI3_RX},
    {.tag = SPI_TAG(SPI_PORT3, RES_SPI_MOSI), .request = DMAMUX_DMAREQ_ID_SPI3_TX},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MISO), .request = DMAMUX_DMAREQ_ID_SPI4_RX},
    {.tag = SPI_TAG(SPI_PORT4, RES_SPI_MOSI), .request = DMAMUX_DMAREQ_ID_SPI4_TX},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR1_CH1},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR1_CH2},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR1_CH3},
    {.tag = TIMER_TAG(TIMER1, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR1_CH4},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR8_CH1},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR8_CH2},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR8_CH3},
    {.tag = TIMER_TAG(TIMER8, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR8_CH4},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR2_CH1},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR2_CH2},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR2_CH3},
    {.tag = TIMER_TAG(TIMER2, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR2_CH4},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR3_CH1},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR3_CH2},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR3_CH3},
    {.tag = TIMER_TAG(TIMER3, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR3_CH4},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR4_CH1},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR4_CH2},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR4_CH3},
    {.tag = TIMER_TAG(TIMER4, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR4_CH4},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR5_CH1},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR5_CH2},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR5_CH3},
    {.tag = TIMER_TAG(TIMER5, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR5_CH4},
    {.tag = TIMER_TAG(TIMER20, TIMER_CH1), .request = DMAMUX_DMAREQ_ID_TMR20_CH1},
    {.tag = TIMER_TAG(TIMER20, TIMER_CH2), .request = DMAMUX_DMAREQ_ID_TMR20_CH2},
    {.tag = TIMER_TAG(TIMER20, TIMER_CH3), .request = DMAMUX_DMAREQ_ID_TMR20_CH3},
    {.tag = TIMER_TAG(TIMER20, TIMER_CH4), .request = DMAMUX_DMAREQ_ID_TMR20_CH4},
};

#define DMA_REQ_ID_MAX (sizeof(dma_req_ids) / sizeof(dma_req_id_t))

static dma_assigment_t dma_assigments[DMA_CHANNEL_MAX] = {};

static dmamux_requst_id_sel_type dma_find_req(resource_tag_t tag) {
  for (uint32_t i = 0; i < DMA_REQ_ID_MAX; i++) {
    const dma_req_id_t *req = &dma_req_ids[i];
    if (req->tag == tag) {
      return req->request;
    }
  }
  return 0;
}

const dma_assigment_t *dma_alloc(resource_tag_t tag, dma_device_t dev) {
  for (uint32_t i = 0; i < DMA_CHANNEL_MAX; i++) {
    dma_assigment_t *ass = &dma_assigments[i];
    const dma_channel_def_t *def = &dma_channel_defs[i];
    if (ass->dev == 0) {
      ass->def = def;
      ass->request = dma_find_req(tag);
      ass->dev = dev;
      return ass;
    }
  }
  return NULL;
}

void dma_enable_rcc(const dma_assigment_t *ass) {
  switch (ass->def->port_index) {
  case 1:
    rcc_enable(RCC_ENCODE(DMA1));
    dmamux_enable(DMA1, TRUE);
    break;
  case 2:
    rcc_enable(RCC_ENCODE(DMA2));
    dmamux_enable(DMA2, TRUE);
    break;
  }
}

static uint32_t dma_flag_for_channel(const dma_channel_def_t *dma, uint32_t val) {
  // 4bits per channel
  const uint32_t shift = (dma->channel_index - 1) * 4;
  const uint32_t port = dma->port_index == 2 ? 0x10000000 : 0x0;
  return port | (val << shift);
}

bool dma_is_flag_active_tc(const dma_assigment_t *ass) {
  const uint32_t flag = dma_flag_for_channel(ass->def, DMA_FDT_FLAG);
  return dma_flag_get(flag);
}

void dma_clear_flag_tc(const dma_assigment_t *ass) {
  dma_flag_clear(dma_flag_for_channel(ass->def, DMA_FDT_FLAG));
  dma_flag_clear(dma_flag_for_channel(ass->def, DMA_HDT_FLAG));
  dma_flag_clear(dma_flag_for_channel(ass->def, DMA_DTERR_FLAG));
}

void dma_prepare_tx_memory(void *addr, uint32_t size) {}
void dma_prepare_rx_memory(void *addr, uint32_t size) {}

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
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(ass);
    break;

  case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
    rgb_dma_isr(ass);
#endif
    break;

  default:
    break;
  }
}

void DMA1_Channel1_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[0]);
}
void DMA1_Channel2_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[1]);
}
void DMA1_Channel3_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[2]);
}
void DMA1_Channel4_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[3]);
}
void DMA1_Channel5_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[4]);
}
void DMA1_Channel6_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[5]);
}
void DMA1_Channel7_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[6]);
}

void DMA2_Channel1_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[7]);
}
void DMA2_Channel2_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[8]);
}
void DMA2_Channel3_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[9]);
}
void DMA2_Channel4_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[10]);
}
void DMA2_Channel5_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[11]);
}
void DMA2_Channel6_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[12]);
}
void DMA2_Channel7_IRQHandler() {
  handle_dma_stream_isr(&dma_assigments[13]);
}