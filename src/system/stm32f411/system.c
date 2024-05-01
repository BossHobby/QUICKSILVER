/**
 *=============================================================================
 *        Supported STM32F40xx/41xx/427x/437x devices
 *-----------------------------------------------------------------------------
 *        System Clock source                    | PLL (HSE)
 *-----------------------------------------------------------------------------
 *        SYSCLK(Hz)                             | 108000000
 *-----------------------------------------------------------------------------
 *        HCLK(Hz)                               | 108000000
 *-----------------------------------------------------------------------------
 *        AHB Prescaler                          | 1
 *-----------------------------------------------------------------------------
 *        APB1 Prescaler                         | 2
 *-----------------------------------------------------------------------------
 *        APB2 Prescaler                         | 2
 *-----------------------------------------------------------------------------
 *        HSE Frequency(Hz)                      | 8000000
 *-----------------------------------------------------------------------------
 *        PLL_M                                  | 8
 *-----------------------------------------------------------------------------
 *        PLL_N                                  | 432
 *-----------------------------------------------------------------------------
 *        PLL_P                                  | 4
 *-----------------------------------------------------------------------------
 *        PLL_Q                                  | 9
 *-----------------------------------------------------------------------------
 *        PLLI2S_N                               | NA
 *-----------------------------------------------------------------------------
 *        PLLI2S_R                               | NA
 *-----------------------------------------------------------------------------
 *        I2S input clock                        | NA
 *-----------------------------------------------------------------------------
 *        VDD(V)                                 | 3.3
 *-----------------------------------------------------------------------------
 *        Main regulator output voltage          | Scale1 mode
 *-----------------------------------------------------------------------------
 *        Flash Latency(WS)                      | 2
 *-----------------------------------------------------------------------------
 *        Prefetch Buffer                        | OFF
 *-----------------------------------------------------------------------------
 *        Instruction cache                      | ON
 *-----------------------------------------------------------------------------
 *        Data cache                             | ON
 *-----------------------------------------------------------------------------
 *=============================================================================
 */

#include "stm32f4xx.h"

/************************* Miscellaneous Configuration ************************/
#define VECT_TAB_OFFSET 0x00 /*!< Vector Table base offset field. \ \ \
                                  This value must be a multiple of 0x200. */
/******************************************************************************/

/************************* PLL Parameters *************************************/
/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M 8
#define PLL_N 432

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P 4

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q 9

/******************************************************************************/

uint32_t SystemCoreClock = 108000000;

const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

static void SetSysClock() {
  /******************************************************************************/
  /*            PLL (clocked by HSE) used as System clock source                */
  /******************************************************************************/
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
    HSEStatus = (uint32_t)0x01;
  } else {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01) {
    /* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    /* HCLK = SYSCLK / 1*/
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    /* PCLK1 = HCLK / 2*/
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    /* Configure the main PLL */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);

    /* Enable the main PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till the main PLL is ready */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }

    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

    /* Select the main PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait till the main PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }

  } else { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }
}

__attribute__((__used__)) void SystemInit() {
  /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
  /* Reset the RCC clock configuration to the default reset state ------------*/
  /* Set HSION bit */
  RCC->CR |= (uint32_t)0x00000001;

  /* Reset CFGR register */
  RCC->CFGR = 0x00000000;

  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (uint32_t)0xFEF6FFFF;

  /* Reset PLLCFGR register */
  RCC->PLLCFGR = 0x24003010;

  /* Reset HSEBYP bit */
  RCC->CR &= (uint32_t)0xFFFBFFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;

  HAL_Init();

  /* Configure the System clock source, PLL Multiplier and Divider factors,
     AHB/APBx prescalers and Flash settings ----------------------------------*/
  SetSysClock();

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif

  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
}
