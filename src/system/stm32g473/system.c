/**
 *   This file configures the system clock as follows:
 *=============================================================================
 *-----------------------------------------------------------------------------
 *        System Clock source                    | HSI
 *-----------------------------------------------------------------------------
 *        SYSCLK(Hz)                             | 168000000
 *-----------------------------------------------------------------------------
 *        HCLK(Hz)                               | 168000000
 *-----------------------------------------------------------------------------
 *        AHB Prescaler                          | 1
 *-----------------------------------------------------------------------------
 *        APB1 Prescaler                         | 1
 *-----------------------------------------------------------------------------
 *        APB2 Prescaler                         | 1
 *-----------------------------------------------------------------------------
 *        PLL_M                                  | 1
 *-----------------------------------------------------------------------------
 *        PLL_N                                  | 21
 *-----------------------------------------------------------------------------
 *        PLL_P                                  | 2
 *-----------------------------------------------------------------------------
 *        PLL_Q                                  | 2
 *-----------------------------------------------------------------------------
 *        PLL_R                                  | 2
 *-----------------------------------------------------------------------------
 *        Require 48MHz for RNG                  | Disabled
 *-----------------------------------------------------------------------------
 *=============================================================================
 */

#include "stm32g4xx.h"

#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif

/************************* Miscellaneous Configuration ************************/
#define VECT_TAB_OFFSET 0x00UL
/******************************************************************************/

uint32_t SystemCoreClock = 168000000;
const uint8_t AHBPrescTable[16] = {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

static void ErrorHandler(void) {
  while (1)
    ;
}

static void SystemClock_Config(void) {
  HAL_InitTick(TICK_INT_PRIORITY);

  // Configure the main internal regulator output voltage
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  // Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    ErrorHandler();
  }

  // Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK) {
    ErrorHandler();
  }

  // Initializes the peripherals clocks
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4 | RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_I2C3 | RCC_PERIPHCLK_I2C4 | RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC345 | RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc345ClockSelection = RCC_ADC345CLKSOURCE_SYSCLK;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    ErrorHandler();
  }

  // Configures CRS
  RCC_CRSInitTypeDef CRSInit = {0};
  CRSInit.Prescaler = RCC_CRS_SYNC_DIV1;
  CRSInit.Source = RCC_CRS_SYNC_SOURCE_USB;
  CRSInit.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  CRSInit.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  CRSInit.ErrorLimitValue = 34;
  CRSInit.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&CRSInit);

  /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
  FLASH->ACR = FLASH_ACR_DBG_SWEN | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_4WS;
}

__attribute__((__used__)) void SystemInit(void) {
/* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << (10 * 2)) | (3UL << (11 * 2))); /* set CP10 and CP11 Full Access */
#endif

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

  HAL_Init();

  SystemClock_Config();
}
