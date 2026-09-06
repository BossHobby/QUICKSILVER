#include "stm32h7xx.h"

#if !defined(HSE_VALUE)
#define HSE_VALUE ((uint32_t)25000000) /*!< Value of the External oscillator in Hz */
#endif                                 /* HSE_VALUE */

#if !defined(CSI_VALUE)
#define CSI_VALUE ((uint32_t)4000000) /*!< Value of the Internal oscillator in Hz*/
#endif                                /* CSI_VALUE */

#if !defined(HSI_VALUE)
#define HSI_VALUE ((uint32_t)64000000) /*!< Value of the Internal oscillator in Hz*/
#endif                                 /* HSI_VALUE */

/************************* Miscellaneous Configuration ************************/
#define VECT_TAB_OFFSET 0x00
/******************************************************************************/

uint32_t SystemCoreClock = 64000000;
uint32_t SystemD2Clock = 64000000;
const uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};

static void ErrorHandler(void) {
  while (1)
    ;
}

typedef struct pllConfig_s {
  uint16_t clockMhz;
  uint8_t m;
  uint16_t n;
  uint8_t p;
  uint8_t q;
  uint8_t r;
  uint32_t vos;
  uint32_t vciRange;
} pllConfig_t;

#if defined(STM32H743xx) || defined(STM32H750xx)
/*
   PLL1 configuration for different silicon revisions of H743 and H750.

   Note for future overclocking support.

   - Rev.Y (and Rev.X), nominal max at 400MHz, runs stably overclocked to 480MHz.
   - Rev.V, nominal max at 480MHz, runs stably at 540MHz, but not to 600MHz (VCO probably out of operating range)

   - A possible frequency table would look something like this, and a revision
     check logic would place a cap for Rev.Y and V.

        400 420 440 460 (Rev.Y & V ends here) 480 500 520 540
 */

// 400MHz for Rev.Y (and Rev.X)
pllConfig_t pll1ConfigRevY = {
    .clockMhz = 400,
    .m = 4,
    .n = 400,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE1,
    .vciRange = RCC_PLL1VCIRANGE_2,
};

// 480MHz for Rev.V
pllConfig_t pll1ConfigRevV = {
    .clockMhz = 480,
    .m = 4,
    .n = 480,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_2,
};

#define MCU_HCLK_DIVIDER RCC_HCLK_DIV2

// H743 and H750
// For HCLK=200MHz with VOS1 range, ST recommended flash latency is 2WS.
// RM0433 (Rev.5) Table 12. FLASH recommended number of wait states and programming delay
//
// For higher HCLK frequency, VOS0 is available on RevV silicons, with FLASH wait states 4WS
// AN5312 (Rev.1) Section 1.2.1 Voltage scaling Table.1
//
// XXX Check if Rev.V requires a different value

#define MCU_FLASH_LATENCY FLASH_LATENCY_2

// Source for CRS input
#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB2

// Workaround for weird HSE behaviors
// (Observed only on Rev.V H750, but may also apply to H743 and Rev.V.)
#define USE_H7_HSERDY_SLOW_WORKAROUND
#define USE_H7_HSE_TIMEOUT_WORKAROUND

#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)

// Nominal max 280MHz with 8MHz HSE
// (340 is okay, 360 doesn't work.)
//

pllConfig_t pll1Config7A3 = {
    .clockMhz = 280,
    .m = 4,
    .n = 280,
    .p = 2,
    .q = 8,
    .r = 5,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_1,
};

// Unlike H743/H750, HCLK can be directly fed with SYSCLK.
#define MCU_HCLK_DIVIDER RCC_HCLK_DIV1

// RM0455 (Rev.6) Table 15. FLASH recommended number of wait states and programming delay
// 280MHz at VOS0 is 6WS

#define MCU_FLASH_LATENCY FLASH_LATENCY_6

// Source for CRS input
#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB1

#elif defined(STM32H723xx) || defined(STM32H725xx)

// Nominal max 550MHz

pllConfig_t pll1Config72x = {
    .clockMhz = 550,
    .m = 4,
    .n = 275,
    .p = 1,
    .q = 2,
    .r = 2,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_1,
};

#define MCU_HCLK_DIVIDER RCC_HCLK_DIV2

// RM0468 (Rev.2) Table 16.
// 550MHz (AXI Interface clock) at VOS0 is 3WS
#define MCU_FLASH_LATENCY FLASH_LATENCY_3

#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB1

#elif defined(STM32H730xx)

// Nominal max 550MHz, but >520Mhz requires ECC to be disabled, CPUFREQ_BOOST set in option bytes and prevents OCTOSPI clock from running at the correct clock speed.
// 4.9.24 FLASH option status register 2 (FLASH_OPTSR2_CUR)
// "Bit 2CPUFREQ_BOOST: CPU frequency boost status bitThis bit indicates whether the CPU frequency can be boosted or not. When it is set, the ECC on ITCM and DTCM are no more used"
// ...
// So use 520Mhz so that OCTOSPI clk can be 200Mhz with OCTOPSI prescaler 2 via PLL2R or 130Mhz with OCTOPSI prescaler 1 via PLL1Q

pllConfig_t pll1Config73x = {
    .clockMhz = 520,
    .m = 2,
    .n = 130,
    .p = 1,
    .q = 4,
    .r = 2,
    .vos = PWR_REGULATOR_VOLTAGE_SCALE0,
    .vciRange = RCC_PLL1VCIRANGE_1,
};

#define MCU_HCLK_DIVIDER RCC_HCLK_DIV2

// RM0468 (Rev.2) Table 16.
// 520MHz (AXI Interface clock) at VOS0 is 3WS
#define MCU_FLASH_LATENCY FLASH_LATENCY_3

#define MCU_RCC_CRS_SYNC_SOURCE RCC_CRS_SYNC_SOURCE_USB1

#else
#error Unknown MCU type
#endif

// HSE clock configuration, originally taken from
// STM32Cube_FW_H7_V1.3.0/Projects/STM32H743ZI-Nucleo/Examples/RCC/RCC_ClockConfig/Src/main.c
static void SystemClockHSE_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

#ifdef notdef
  // CSI has been disabled at SystemInit().
  // HAL_RCC_ClockConfig() will fail because CSIRDY is off.

  /* -1- Select CSI as system clock source to allow modification of the PLL configuration */

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_CSI;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    /* Initialization Error */
    ErrorHandler();
  }
#endif

  pllConfig_t *pll1Config;

#if defined(STM32H743xx) || defined(STM32H750xx)
  pll1Config = (HAL_GetREVID() == REV_ID_V) ? &pll1ConfigRevV : &pll1ConfigRevY;
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
  pll1Config = &pll1Config7A3;
#elif defined(STM32H723xx) || defined(STM32H725xx)
  pll1Config = &pll1Config72x;
#elif defined(STM32H730xx)
  pll1Config = &pll1Config73x;
#else
#error Unknown MCU type
#endif

  // Configure voltage scale.
  // It has been pre-configured at PWR_REGULATOR_VOLTAGE_SCALE1,
  // and it may stay or overridden by PWR_REGULATOR_VOLTAGE_SCALE0 depending on the clock config.

  __HAL_PWR_VOLTAGESCALING_CONFIG(pll1Config->vos);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    // Empty
  }

  /* -2- Enable HSE  Oscillator, select it as PLL source and finally activate the PLL */

#ifdef USE_H7_HSERDY_SLOW_WORKAROUND
  // With reference to 2.3.22 in the ES0250 Errata for the L476.
  // Applying the same workaround here in the vain hopes that it improves startup times.
  // Randomly the HSERDY bit takes AGES, over 10 seconds, to be set.

  __HAL_RCC_GPIOH_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio_initstruct;
  gpio_initstruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_initstruct.Pull = GPIO_NOPULL;
  gpio_initstruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOH, &gpio_initstruct);
#endif

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Even Nucleo-H473ZI and Nucleo-H7A3ZI work without RCC_HSE_BYPASS

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = pll1Config->m;
  RCC_OscInitStruct.PLL.PLLN = pll1Config->n;
  RCC_OscInitStruct.PLL.PLLP = pll1Config->p;
  RCC_OscInitStruct.PLL.PLLQ = pll1Config->q;
  RCC_OscInitStruct.PLL.PLLR = pll1Config->r;

  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLRGE = pll1Config->vciRange;
  HAL_StatusTypeDef status = HAL_RCC_OscConfig(&RCC_OscInitStruct);

#ifdef USE_H7_HSE_TIMEOUT_WORKAROUND
  if (status == HAL_TIMEOUT) {
    NVIC_SystemReset(); // DC - sometimes HSERDY gets stuck, waiting longer doesn't help.
  }
#endif

  if (status != HAL_OK) {
    /* Initialization Error */
    ErrorHandler();
  }

  // Configure PLL2 and PLL3
  // Use of PLL2 and PLL3 are not determined yet.
  // A review of total system wide clock requirements is necessary.

  // Configure SCGU (System Clock Generation Unit)
  // Select PLL as system clock source and configure bus clock dividers.
  //
  // Clock type and divider member names do not have direct visual correspondence.
  // Here is how these correspond:
  //   RCC_CLOCKTYPE_SYSCLK           sys_ck
  //   RCC_CLOCKTYPE_HCLK             AHBx (rcc_hclk1,rcc_hclk2,rcc_hclk3,rcc_hclk4)
  //   RCC_CLOCKTYPE_D1PCLK1          APB3 (rcc_pclk3)
  //   RCC_CLOCKTYPE_PCLK1            APB1 (rcc_pclk1)
  //   RCC_CLOCKTYPE_PCLK2            APB2 (rcc_pclk2)
  //   RCC_CLOCKTYPE_D3PCLK1          APB4 (rcc_pclk4)

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
                                 RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_D1PCLK1 |
                                 RCC_CLOCKTYPE_PCLK1 |
                                 RCC_CLOCKTYPE_PCLK2 |
                                 RCC_CLOCKTYPE_D3PCLK1);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;

  RCC_ClkInitStruct.AHBCLKDivider = MCU_HCLK_DIVIDER;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, MCU_FLASH_LATENCY) != HAL_OK) {
    /* Initialization Error */
    ErrorHandler();
  }

  /* -4- Optional: Disable CSI Oscillator (if the HSI is no more needed by the application)*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    ErrorHandler();
  }
}

void SystemClock_Config(void) {
  // Configure power supply

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H7A3xx) || defined(STM32H730xx)
  // Legacy H7 devices (H743, H750) and newer but SMPS-less devices(H7A3, H723, H730)

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  // Pre-configure voltage scale to PWR_REGULATOR_VOLTAGE_SCALE1.
  // SystemClockHSE_Config may configure PWR_REGULATOR_VOLTAGE_SCALE0.

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#elif defined(STM32H7A3xxQ) || defined(STM32H725xx)

  // We assume all SMPS equipped devices use this mode (Direct SMPS).
  // - All STM32H7A3xxQ devices.
  // - All STM32H725xx devices (Note STM32H725RG is Direct SMPS only - no LDO).
  //
  // Note that:
  // - Nucleo-H7A3ZI-Q is preconfigured for power supply configuration 2 (Direct SMPS).
  // - Nucleo-H723ZI-Q transplanted with STM32H725ZG is the same as above.

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

#else
#error Unknown MCU
#endif

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    // Empty
  }

  SystemClockHSE_Config();

  /*activate CSI clock mondatory for I/O Compensation Cell*/

  __HAL_RCC_CSI_ENABLE();

  /* Enable SYSCFG clock mondatory for I/O Compensation Cell */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Enables the I/O Compensation Cell */

  HAL_EnableCompensationCell();

  HAL_Delay(10);

  // Configure peripheral clocks

  RCC_PeriphCLKInitTypeDef RCC_PeriphClkInit;

  // Configure HSI48 as peripheral clock for USB

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  RCC_PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  // Configure CRS for dynamic calibration of HSI48
  // While ES0392 Rev 5 "STM32H742xI/G and STM32H743xI/G device limitations" states CRS not working for REV.Y,
  // it is always turned on as it seems that it has no negative effect on clock accuracy.

  RCC_CRSInitTypeDef crsInit = {
      .Prescaler = RCC_CRS_SYNC_DIV1,
      .Source = MCU_RCC_CRS_SYNC_SOURCE,
      .Polarity = RCC_CRS_SYNC_POLARITY_RISING,
      .ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT,
      .ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT,
      .HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT,
  };

  __HAL_RCC_CRS_CLK_ENABLE();
  HAL_RCCEx_CRSConfig(&crsInit);

  // Configure UART peripheral clock sources
  //
  // Possible sources:
  //   D2PCLK1 (pclk1 for APB1 = USART234578)
  //   D2PCLK2 (pclk2 for APB2 = USART16)
  //   PLL2 (pll2_q_ck)
  //   PLL3 (pll3_q_ck),
  //   HSI (hsi_ck),
  //   CSI (csi_ck),LSE(lse_ck);

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART16 | RCC_PERIPHCLK_USART234578;
  RCC_PeriphClkInit.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  RCC_PeriphClkInit.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  // Configure SPI peripheral clock sources
  //
  // Possible sources for SPI123:
  //   PLL (pll1_q_ck)
  //   PLL2 (pll2_p_ck)
  //   PLL3 (pll3_p_ck)
  //   PIN (I2S_CKIN)
  //   CLKP (per_ck)
  // Possible sources for SPI45:
  //   D2PCLK1 (rcc_pclk2 = APB1) 100MHz
  //   PLL2 (pll2_q_ck)
  //   PLL3 (pll3_q_ck)
  //   HSI (hsi_ker_ck)
  //   CSI (csi_ker_ck)
  //   HSE (hse_ck)
  // Possible sources for SPI6:
  //   D3PCLK1 (rcc_pclk4 = APB4) 100MHz
  //   PLL2 (pll2_q_ck)
  //   PLL3 (pll3_q_ck)
  //   HSI (hsi_ker_ck)
  //   CSI (csi_ker_ck)
  //   HSE (hse_ck)

  // We use 100MHz for Rev.Y and 120MHz for Rev.V from various sources

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SPI123 | RCC_PERIPHCLK_SPI45 | RCC_PERIPHCLK_SPI6;
  RCC_PeriphClkInit.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  RCC_PeriphClkInit.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  RCC_PeriphClkInit.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  // Configure I2C peripheral clock sources
  //
  // Current source for I2C123:
  //   D2PCLK1 (rcc_pclk1 = APB1 peripheral clock)
  //
  // Current source for I2C4:
  //   D3PCLK1 (rcc_pclk4 = APB4 peripheral clock)
  //
  // Note that peripheral clock determination in bus_i2c_hal_init.c must be modified when the sources are modified.

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C123 | RCC_PERIPHCLK_I2C4;
  RCC_PeriphClkInit.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  RCC_PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

#ifdef USE_SDCARD_SDIO
  __HAL_RCC_SDMMC1_CLK_ENABLE(); // FIXME enable SDMMC1 or SDMMC2 depending on target.

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;

#if (HSE_VALUE != 8000000)
#error Unsupported external oscillator speed.  The calculations below are based on 8Mhz resonators
// if you are seeing this, then calculate the PLL2 settings for your resonator and add support as required.
#else
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H7A3xx) || defined(STM32H7A3xxQ) || defined(STM32H725xx)
  RCC_PeriphClkInit.PLL2.PLL2M = 5;
  RCC_PeriphClkInit.PLL2.PLL2N = 500;                  // 8Mhz (Oscillator Frequency) / 5 (PLL2M) = 1.6 * 500 (PLL2N) = 800Mhz.
  RCC_PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE; // Wide VCO range:192 to 836 MHz
  RCC_PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0; // PLL2 input between 1 and 2Mhz (1.6)
  RCC_PeriphClkInit.PLL2.PLL2FRACN = 0;

  RCC_PeriphClkInit.PLL2.PLL2P = 2; // 800Mhz / 2 = 400Mhz
  RCC_PeriphClkInit.PLL2.PLL2Q = 3; // 800Mhz / 3 = 266Mhz // 133Mhz can be derived from this for for QSPI if flash chip supports the speed.
  RCC_PeriphClkInit.PLL2.PLL2R = 4; // 800Mhz / 4 = 200Mhz // HAL LIBS REQUIRE 200MHZ SDMMC CLOCK, see HAL_SD_ConfigWideBusOperation, SDMMC_HSpeed_CLK_DIV, SDMMC_NSpeed_CLK_DIV
#elif defined(STM32H730xx)
  RCC_PeriphClkInit.PLL2.PLL2M = 8;
  RCC_PeriphClkInit.PLL2.PLL2N = 400;                    // 8Mhz (Oscillator Frequency) / 8 (PLL2M) = 1.0 * 400 (PLL2N) = 400Mhz.
  RCC_PeriphClkInit.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM; // Medium VCO range:150 to 420 MHz
  RCC_PeriphClkInit.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;   // PLL2 input between 1 and 2Mhz (1.0)
  RCC_PeriphClkInit.PLL2.PLL2FRACN = 0;

  RCC_PeriphClkInit.PLL2.PLL2P = 3; // 400Mhz / 3 = 133Mhz // ADC does't like much higher when using PLL2P
  RCC_PeriphClkInit.PLL2.PLL2Q = 3; // 400Mhz / 3 = 133Mhz // SPI6 does't like much higher when using PLL2Q
  RCC_PeriphClkInit.PLL2.PLL2R = 2; // 400Mhz / 2 = 200Mhz // HAL LIBS REQUIRE 200MHZ SDMMC CLOCK, see HAL_SD_ConfigWideBusOperation, SDMMC_HSpeed_CLK_DIV, SDMMC_NSpeed_CLK_DIV
#else
#error Unknown MCU type
#endif
  RCC_PeriphClkInit.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);
#endif
#endif

  RCC_PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  RCC_PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphClkInit);

  // TODO H730 OCTOSPI clock for 100Mhz flash chips should use PLL2R at 200Mhz

  // Configure MCO clocks for clock test/verification

  // Possible sources for MCO1:
  //   RCC_MCO1SOURCE_HSI (hsi_ck)
  //   RCC_MCO1SOURCE_LSE (?)
  //   RCC_MCO1SOURCE_HSE (hse_ck)
  //   RCC_MCO1SOURCE_PLL1QCLK (pll1_q_ck)
  //   RCC_MCO1SOURCE_HSI48 (hsi48_ck)

  //  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);     // HSE(8M) / 1 = 1M
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI48, RCC_MCODIV_4); // HSI48(48M) / 4 = 12M

  // Possible sources for MCO2:
  //   RCC_MCO2SOURCE_SYSCLK  (sys_ck)
  //   RCC_MCO2SOURCE_PLL2PCLK (pll2_p_ck)
  //   RCC_MCO2SOURCE_HSE (hse_ck)
  //   RCC_MCO2SOURCE_PLLCLK (pll1_p_ck)
  //   RCC_MCO2SOURCE_CSICLK (csi_ck)
  //   RCC_MCO2SOURCE_LSICLK (lsi_ck)

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLCLK, RCC_MCODIV_15); // PLL1P(400M) / 15 = 26.67M
}

__attribute__((__used__)) void SystemInit(void) {
  // FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); // Set CP10 and CP11 Full Access
#endif

  // Reset the RCC clock configuration to the default reset state
  // Set HSION bit
  RCC->CR = RCC_CR_HSION;

  // Reset CFGR register
  RCC->CFGR = 0x00000000;

  // Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON bits

  // XXX Don't do this until we are established with clock handling
  // RCC->CR &= (uint32_t)0xEAF6ED7F;

  // Instead, we explicitly turn those on
  RCC->CR |= RCC_CR_CSION;
  RCC->CR |= RCC_CR_HSION;
  RCC->CR |= RCC_CR_HSEON;
  RCC->CR |= RCC_CR_HSI48ON;

#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
  /* Reset D1CFGR register */
  RCC->D1CFGR = 0x00000000;

  /* Reset D2CFGR register */
  RCC->D2CFGR = 0x00000000;

  /* Reset D3CFGR register */
  RCC->D3CFGR = 0x00000000;
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
  /* Reset CDCFGR1 register */
  RCC->CDCFGR1 = 0x00000000;

  /* Reset CDCFGR2 register */
  RCC->CDCFGR2 = 0x00000000;

  /* Reset SRDCFGR register */
  RCC->SRDCFGR = 0x00000000;
#endif

  /* Reset PLLCKSELR register */
  RCC->PLLCKSELR = 0x00000000;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x00000000;
  /* Reset PLL1DIVR register */
  RCC->PLL1DIVR = 0x00000000;
  /* Reset PLL1FRACR register */
  RCC->PLL1FRACR = 0x00000000;

  /* Reset PLL2DIVR register */
  RCC->PLL2DIVR = 0x00000000;

  /* Reset PLL2FRACR register */

  RCC->PLL2FRACR = 0x00000000;
  /* Reset PLL3DIVR register */
  RCC->PLL3DIVR = 0x00000000;

  /* Reset PLL3FRACR register */
  RCC->PLL3FRACR = 0x00000000;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIER = 0x00000000;

  /* Change  the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7) */
  *((__IO uint32_t *)0x51008108) = 0x00000001;

  /* Configure the Vector Table location add offset address ------------------*/
#if defined(VECT_TAB_SRAM)
#if defined(STM32H743xx) || defined(STM32H750xx) || defined(STM32H723xx) || defined(STM32H725xx) || defined(STM32H730xx)
  SCB->VTOR = D1_AXISRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal ITCMSRAM */
#elif defined(STM32H7A3xx) || defined(STM32H7A3xxQ)
  SCB->VTOR = CD_AXISRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal ITCMSRAM */
#else
#error Unknown MCU type
#endif
#elif defined(USE_EXST)
  extern uint8_t isr_vector_table_base;

  SCB->VTOR = (uint32_t)&isr_vector_table_base;
#if defined(STM32H730xx)
  /* Configure the Vector Table location add offset address ------------------*/

  extern uint8_t isr_vector_table_flash_base;
  extern uint8_t isr_vector_table_end;

  extern uint8_t ram_isr_vector_table_base;

  memcpy(&ram_isr_vector_table_base, &isr_vector_table_flash_base, (size_t)(&isr_vector_table_end - &isr_vector_table_base));

  SCB->VTOR = (uint32_t)&ram_isr_vector_table_base;
#endif
#else
  SCB->VTOR = FLASH_BANK1_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

  HAL_Init();

  SystemClock_Config();

  // Enable CPU L1-Cache
  SCB_EnableICache();
  SCB_EnableDCache();
}
