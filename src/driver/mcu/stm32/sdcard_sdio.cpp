#include "core/project.h"

#if defined(STM32H7) && defined(USE_SDCARD)

#include <stm32h7xx_ll_rcc.h>
#include <stm32h7xx_ll_sdmmc.h>
#include <string.h>

#include "driver/blackbox/sdcard.h"
#include "driver/gpio.h"

static struct {
  SDMMC_TypeDef *instance;
  uint32_t kernel_clock;
  bool command_pending;
  struct {
    uint8_t *buffer;
    bool read;
    bool active;
  } data;
  // IDMA cannot access DTCM. Reserve whole cache lines in AXI SRAM.
  alignas(32) uint8_t dma_buffer[SDCARD_PAGE_SIZE];
} sdcard;

static void sdcard_sdio_abort() {
  if (!sdcard.instance)
    return;
  sdcard.instance->IDMACTRL = 0;
  sdcard.instance->DCTRL = 0;
  sdcard.instance->CMD = 0;
  sdcard.instance->POWER = 0;
  sdcard.command_pending = false;
  sdcard.data.active = false;
}

static bool sdcard_sdio_init() {
  if (!target_sdcard_sdio_valid())
    return false;
  const auto &port = target.sdio_ports[target.sdcard.sdio];
  const gpio_pins_t pins[] = {port.clk, port.cmd, port.d0, port.d1, port.d2, port.d3};
  const resource_sdio_t signals[] = {RES_SDIO_CLK, RES_SDIO_CMD, RES_SDIO_D0, RES_SDIO_D1, RES_SDIO_D2, RES_SDIO_D3};
  for (uint32_t i = 0; i < 6; i++) {
    if (!gpio_pin_has_tag(pins[i], SDIO_TAG(port.index, signals[i])))
      return false;
  }
  sdcard = {};

  if (port.index == 1) {
    sdcard.instance = SDMMC1;
    LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_SDMMC1);
    LL_AHB3_GRP1_ForceReset(LL_AHB3_GRP1_PERIPH_SDMMC1);
    LL_AHB3_GRP1_ReleaseReset(LL_AHB3_GRP1_PERIPH_SDMMC1);
  } else {
    sdcard.instance = SDMMC2;
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_SDMMC2);
    LL_AHB2_GRP1_ForceReset(LL_AHB2_GRP1_PERIPH_SDMMC2);
    LL_AHB2_GRP1_ReleaseReset(LL_AHB2_GRP1_PERIPH_SDMMC2);
  }
  // Reuse PLL1Q without retuning clocks shared with other peripherals.
  LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC_CLKSOURCE_PLL1Q);
  sdcard.kernel_clock = LL_RCC_GetSDMMCClockFreq(LL_RCC_SDMMC_CLKSOURCE);
  if (!sdcard.kernel_clock)
    return false;

  gpio_config_t config = {
      .mode = GPIO_ALTERNATE,
      .output = GPIO_PUSHPULL,
      .drive = GPIO_DRIVE_HIGH,
      .pull = GPIO_UP_PULL,
  };
  for (uint32_t i = 0; i < 6; i++) {
    config.pull = signals[i] == RES_SDIO_CLK ? GPIO_NO_PULL : GPIO_UP_PULL;
    gpio_pin_init_tag(pins[i], config, SDIO_TAG(port.index, signals[i]));
  }

  // Completion is polled by the scheduler, not an interrupt handler.
  sdcard.instance->MASK = 0;
  sdcard.instance->CLKCR = (sdcard.kernel_clock + 799999) / 800000; // At most 400 kHz.
  sdcard.instance->POWER = SDMMC_POWER_PWRCTRL;
  return true;
}

static void sdcard_sdio_configure() {
  const uint32_t divider = (sdcard.kernel_clock + 49999999) / 50000000;
  sdcard.instance->CLKCR = divider | SDMMC_CLKCR_WIDBUS_0 | SDMMC_CLKCR_HWFC_EN;
}

static sdcard_transfer_status_t sdcard_sdio_command(uint8_t index, uint32_t argument, sdcard_response_t response, sdcard_response_data_t *result) {
  if (!sdcard.command_pending) {
    uint32_t wait = SDMMC_CMD_WAITRESP_0;
    if (response == SDCARD_RESPONSE_NONE)
      wait = 0;
    else if (response == SDCARD_RESPONSE_R2)
      wait = SDMMC_CMD_WAITRESP;

    sdcard.instance->ICR = SDMMC_STATIC_CMD_FLAGS;
    sdcard.instance->ARG = argument;
    sdcard.instance->CMD = index | wait | SDMMC_CMD_CPSMEN | (sdcard.data.active ? SDMMC_CMD_CMDTRANS : 0);
    sdcard.command_pending = true;
    return SDCARD_TRANSFER_WAIT;
  }

  const uint32_t status = sdcard.instance->STA;
  if (status & SDMMC_STA_CTIMEOUT) {
    sdcard.command_pending = false;
    sdcard.instance->CMD = 0;
    return SDCARD_TRANSFER_TIMEOUT;
  }
  if ((status & SDMMC_STA_CCRCFAIL) && response != SDCARD_RESPONSE_R3) {
    sdcard.command_pending = false;
    return SDCARD_TRANSFER_ERROR;
  }

  uint32_t completed = SDMMC_STA_CMDREND;
  if (response == SDCARD_RESPONSE_NONE)
    completed = SDMMC_STA_CMDSENT;
  else if (response == SDCARD_RESPONSE_R3)
    completed |= SDMMC_STA_CCRCFAIL; // OCR has no CRC.

  if (!(status & completed))
    return SDCARD_TRANSFER_WAIT;
  sdcard.command_pending = false;

  if (response == SDCARD_RESPONSE_R1 && (sdcard.instance->RESP1 & SDMMC_OCR_ERRORBITS))
    return SDCARD_TRANSFER_ERROR;
  if (response == SDCARD_RESPONSE_R6 && (sdcard.instance->RESP1 & 0xe000))
    return SDCARD_TRANSFER_ERROR;
  const bool has_command_index = response == SDCARD_RESPONSE_R1 || response == SDCARD_RESPONSE_R6 || response == SDCARD_RESPONSE_R7;
  if (has_command_index && (sdcard.instance->RESPCMD & 0x3f) != index)
    return SDCARD_TRANSFER_ERROR;
  result->status = sdcard.instance->RESP1;
  result->words[0] = sdcard.instance->RESP1;
  result->words[1] = sdcard.instance->RESP2;
  result->words[2] = sdcard.instance->RESP3;
  result->words[3] = sdcard.instance->RESP4;
  return SDCARD_TRANSFER_DONE;
}

static void sdcard_sdio_data_prepare(uint8_t *buffer, uint32_t size, bool read) {
  sdcard.data = {.buffer = buffer, .read = read, .active = true};
  if (read) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sdcard.dma_buffer, sizeof(sdcard.dma_buffer));
  } else {
    memcpy(sdcard.dma_buffer, buffer, size);
    SCB_CleanDCache_by_Addr((uint32_t *)sdcard.dma_buffer, sizeof(sdcard.dma_buffer));
  }
  __DSB();
  sdcard.instance->ICR = SDMMC_STATIC_DATA_FLAGS;
  sdcard.instance->DTIMER = sdcard.kernel_clock;
  sdcard.instance->DLEN = size;
  sdcard.instance->DCTRL = (9 << SDMMC_DCTRL_DBLOCKSIZE_Pos) | (read ? SDMMC_DCTRL_DTDIR : 0);
  sdcard.instance->IDMABASE0 = (uint32_t)sdcard.dma_buffer;
  sdcard.instance->IDMACTRL = SDMMC_IDMA_IDMAEN;
}

static sdcard_transfer_status_t sdcard_sdio_data_poll() {
  const uint32_t status = sdcard.instance->STA;
  const uint32_t errors = SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR | SDMMC_STA_IDMATE;
  if (status & errors)
    return SDCARD_TRANSFER_ERROR;
  if (!(status & SDMMC_STA_DATAEND))
    return SDCARD_TRANSFER_WAIT;
  sdcard.instance->IDMACTRL = 0;
  sdcard.instance->DCTRL = 0;
  sdcard.instance->CMD = 0;
  sdcard.data.active = false;
  if (sdcard.data.read) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)sdcard.dma_buffer, sizeof(sdcard.dma_buffer));
    __DSB();
    memcpy(sdcard.data.buffer, sdcard.dma_buffer, sizeof(sdcard.dma_buffer));
  }
  return SDCARD_TRANSFER_DONE;
}

const sdcard_transport_t sdcard_sdio = {
    .spi = false,
    .init = sdcard_sdio_init,
    .abort = sdcard_sdio_abort,
    .configure = sdcard_sdio_configure,
    .command = sdcard_sdio_command,
    .data_prepare = sdcard_sdio_data_prepare,
    .data_poll = sdcard_sdio_data_poll,
};
#endif
