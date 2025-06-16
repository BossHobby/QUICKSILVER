#include "driver/dma.h"

#include "core/failloop.h"
#include "core/target.h"
#include "driver/adc.h"
#include "driver/rcc.h"
#include "driver/resource.h"
#include "driver/timer.h"

#ifdef STM32F4
#ifdef USE_MOTOR_DSHOT
#include "driver/motor_dshot.h"
#endif
#include "driver/spi.h"
#endif

#if defined(STM32F7) || defined(STM32H7)
#define CACHE_LINE_SIZE 32
#define CACHE_LINE_MASK (CACHE_LINE_SIZE - 1)
#endif

#if defined(STM32G4)
#define DMA_STREAM(_port, _stream)                 \
  [DMA##_port##_STREAM##_stream] = {               \
      .port = DMA##_port,                          \
      .port_index = _port,                         \
      .stream = DMA##_port##_Channel##_stream,     \
      .stream_index = LL_DMA_CHANNEL_##_stream,    \
      .irq = DMA##_port##_Channel##_stream##_IRQn, \
  },
#else
#define DMA_STREAM(_port, _stream)                \
  [DMA##_port##_STREAM##_stream] = {              \
      .port = DMA##_port,                         \
      .port_index = _port,                        \
      .stream = DMA##_port##_Stream##_stream,     \
      .stream_index = LL_DMA_STREAM_##_stream,    \
      .irq = DMA##_port##_Stream##_stream##_IRQn, \
  },
#endif
const dma_stream_def_t dma_stream_defs[DMA_STREAM_MAX] = {DMA_STREAMS};
#undef DMA_STREAM

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

void dma_enable_rcc(const dma_stream_def_t *def) {
#ifdef STM32G4
  rcc_enable(RCC_AHB1_GRP1(DMAMUX1));
#endif

  switch (def->port_index) {
  case 1:
    rcc_enable(RCC_AHB1_GRP1(DMA1));
    break;
  case 2:
    rcc_enable(RCC_AHB1_GRP1(DMA2));
    break;
  }
}

#if !defined(STM32H7) && !defined(STM32G4)
uint32_t dma_map_channel(uint32_t channel) {
  switch (channel) {
  case 0:
    return LL_DMA_CHANNEL_0;
  case 1:
    return LL_DMA_CHANNEL_1;
  case 2:
    return LL_DMA_CHANNEL_2;
  case 3:
    return LL_DMA_CHANNEL_3;
  case 4:
    return LL_DMA_CHANNEL_4;
  case 5:
    return LL_DMA_CHANNEL_5;
  case 6:
    return LL_DMA_CHANNEL_6;
  case 7:
    return LL_DMA_CHANNEL_7;
  default:
    failloop(FAILLOOP_DMA);
    return 0;
  }
}
#endif

#ifdef STM32F4
// STM32F4 fixed DMA stream-channel assignments based on AN4031
typedef struct {
  dma_device_t device;
  dma_stream_t stream;
  uint32_t channel;
} stm32f4_dma_assignment_t;

static const stm32f4_dma_assignment_t stm32f4_dma_assignments[] = {
    // USART1
    {DMA_DEVICE_SERIAL1_RX, DMA2_STREAM2, 4}, // Primary option
    {DMA_DEVICE_SERIAL1_RX, DMA2_STREAM5, 4}, // Alternative option
    {DMA_DEVICE_SERIAL1_TX, DMA2_STREAM7, 4},

    // USART2
    {DMA_DEVICE_SERIAL2_RX, DMA1_STREAM5, 4},
    {DMA_DEVICE_SERIAL2_TX, DMA1_STREAM6, 4},

    // USART3
    {DMA_DEVICE_SERIAL3_RX, DMA1_STREAM1, 4},
    {DMA_DEVICE_SERIAL3_TX, DMA1_STREAM3, 4}, // Primary option
    {DMA_DEVICE_SERIAL3_TX, DMA1_STREAM4, 7}, // Alternative option

    // UART4
    {DMA_DEVICE_SERIAL4_RX, DMA1_STREAM2, 4},
    {DMA_DEVICE_SERIAL4_TX, DMA1_STREAM4, 4},

    // UART5
    {DMA_DEVICE_SERIAL5_RX, DMA1_STREAM0, 4},
    {DMA_DEVICE_SERIAL5_TX, DMA1_STREAM7, 4},

    // USART6
    {DMA_DEVICE_SERIAL6_RX, DMA2_STREAM1, 5}, // Primary option
    {DMA_DEVICE_SERIAL6_RX, DMA2_STREAM2, 5}, // Alternative option
    {DMA_DEVICE_SERIAL6_TX, DMA2_STREAM6, 5}, // Primary option
    {DMA_DEVICE_SERIAL6_TX, DMA2_STREAM7, 5}, // Alternative option
};

bool dma_allocate_stream(dma_device_t device, uint32_t channel, resource_tag_t tag) {
  if (target.dma[device].dma != DMA_STREAM_INVALID) {
    return true;
  }

  // STM32F4 uses fixed stream-channel assignments for serial devices
  if (device >= DMA_DEVICE_SERIAL1_RX && device <= DMA_DEVICE_SERIAL8_TX) {
    for (uint32_t i = 0; i < sizeof(stm32f4_dma_assignments) / sizeof(stm32f4_dma_assignment_t); i++) {
      const stm32f4_dma_assignment_t *assignment = &stm32f4_dma_assignments[i];

      if (assignment->device == device && assignment->channel == channel) {
        // Check if this stream is available
        if (dma_stream_map[assignment->stream] == DMA_DEVICE_INVALID) {
          dma_stream_map[assignment->stream] = device;
          target.dma[device].dma = assignment->stream;
          target.dma[device].channel = assignment->channel;
          target.dma[device].tag = tag;
          return true;
        }
      }
    }
    return false;
  }

  // For non-serial devices, use simple allocation
  for (uint32_t i = 1; i < DMA_STREAM_MAX; i++) {
    if (dma_stream_map[i] == DMA_DEVICE_INVALID) {
      dma_stream_map[i] = device;
      target.dma[device].dma = (dma_stream_t)i;
      target.dma[device].channel = channel;
      target.dma[device].tag = tag;
      return true;
    }
  }

  return false;
}
#endif

extern void dshot_dma_isr(const dma_device_t);
extern void spi_dma_isr(const dma_device_t);
extern void rgb_dma_isr(const dma_device_t);
extern void serial_dma_isr(const dma_device_t);

static void handle_dma_stream_isr(const dma_device_t dev) {
  switch (dev) {
  case DMA_DEVICE_SPI1_RX:
  case DMA_DEVICE_SPI2_RX:
  case DMA_DEVICE_SPI3_RX:
  case DMA_DEVICE_SPI4_RX:
  case DMA_DEVICE_SPI1_TX:
  case DMA_DEVICE_SPI2_TX:
  case DMA_DEVICE_SPI3_TX:
  case DMA_DEVICE_SPI4_TX:
    spi_dma_isr(dev);
    break;
  case DMA_DEVICE_DSHOT_CH1:
  case DMA_DEVICE_DSHOT_CH2:
  case DMA_DEVICE_DSHOT_CH3:
#ifdef USE_MOTOR_DSHOT
    dshot_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_SERIAL1_RX:
  case DMA_DEVICE_SERIAL1_TX:
  case DMA_DEVICE_SERIAL2_RX:
  case DMA_DEVICE_SERIAL2_TX:
  case DMA_DEVICE_SERIAL3_RX:
  case DMA_DEVICE_SERIAL3_TX:
  case DMA_DEVICE_SERIAL4_RX:
  case DMA_DEVICE_SERIAL4_TX:
  case DMA_DEVICE_SERIAL5_RX:
  case DMA_DEVICE_SERIAL5_TX:
  case DMA_DEVICE_SERIAL6_RX:
  case DMA_DEVICE_SERIAL6_TX:
  case DMA_DEVICE_SERIAL7_RX:
  case DMA_DEVICE_SERIAL7_TX:
  case DMA_DEVICE_SERIAL8_RX:
  case DMA_DEVICE_SERIAL8_TX:
#ifdef USE_SERIAL
    serial_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_RGB:
#ifdef USE_RGB_LED
    rgb_dma_isr(dev);
#endif
    break;
  case DMA_DEVICE_INVALID:
  case DMA_DEVICE_MAX:
    break;
  }
}

#define DMA_STREAM(_port, _stream)                                                                                          \
  void DMA##_port##_Stream##_stream##_IRQHandler() { handle_dma_stream_isr(dma_stream_map[DMA##_port##_STREAM##_stream]); } \
  void DMA##_port##_Channel##_stream##_IRQHandler() { handle_dma_stream_isr(dma_stream_map[DMA##_port##_STREAM##_stream]); }

DMA_STREAMS

#undef DMA_STREAM

#ifdef STM32F4
// STM32F4 errata 2.2.19: centralized DMA2 conflict check
bool dma_can_use_dma2(dma_device_t device) {
  // If device is valid, check if it uses DMA2
  if (device != DMA_DEVICE_INVALID) {
    const dma_stream_def_t *dma = &dma_stream_defs[target.dma[device].dma];
    if (dma->port_index != 2) {
      return true; // Not DMA2, no conflict possible
    }
  }

#ifdef USE_MOTOR_DSHOT
  // DSHOT bitbang always uses DMA2 for GPIO writes which triggers the errata
  if (target.brushless && dshot_phase != 0) {
    return false; // DSHOT bitbang active, DMA2 conflict exists
  }
#endif

  // Check if SPI1 is busy (APB2 peripheral that uses DMA2)
  const dma_stream_def_t *spi1_dma = &dma_stream_defs[target.dma[DMA_DEVICE_SPI1_TX].dma];
  if (spi1_dma->port_index == 2 && !spi_dma_is_ready(SPI_PORT1)) {
    return false;
  }

  return true;
}
#endif