#include "at32f435_437.h"

/**
 * @brief  system clock config program
 * @note   the system clock is configured as follow:
 *         - system clock        = (hext * pll_ns)/(pll_ms * pll_fr)
 *         - system clock source = pll (hext)
 *         - hext                = 8000000
 *         - sclk                = 288000000
 *         - ahbdiv              = 1
 *         - ahbclk              = 288000000
 *         - apb2div             = 2
 *         - apb2clk             = 144000000
 *         - apb1div             = 2
 *         - apb1clk             = 144000000
 *         - pll_ns              = 72
 *         - pll_ms              = 1
 *         - pll_fr              = 2
 * @param  none
 * @retval none
 */
void system_clock_config(void) {
  /* enable pwc periph clock */
  crm_periph_clock_enable(CRM_PWC_PERIPH_CLOCK, TRUE);

  /* config ldo voltage */
  pwc_ldo_output_voltage_set(PWC_LDO_OUTPUT_1V3);

  /* set the flash clock divider */
  flash_clock_divider_set(FLASH_CLOCK_DIV_3);

  /* reset crm */
  crm_reset();

  /* enable hext */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HEXT, TRUE);

  /* wait till hext is ready */
  while (crm_hext_stable_wait() == ERROR) {
  }

  /* enable hick */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);

  /* wait till hick is ready */
  while (crm_flag_get(CRM_HICK_STABLE_FLAG) != SET) {
  }

  /* config pll clock resource
  common frequency config list: pll source selected  hick or hext(8mhz)
  _______________________________________________________________________________________
  |        |         |         |         |         |         |         |        |        |
  |pll(mhz)|   288   |   252   |   216   |   180   |   144   |   108   |   72   |   36   |
  |________|_________|_________|_________|_________|_________|_________|_________________|
  |        |         |         |         |         |         |         |        |        |
  |pll_ns  |   72    |   63    |   108   |   90    |   72    |   108   |   72   |   72   |
  |        |         |         |         |         |         |         |        |        |
  |pll_ms  |   1     |   1     |   1     |   1     |   1     |   1     |   1    |   1    |
  |        |         |         |         |         |         |         |        |        |
  |pll_fr  |   FR_2  |   FR_2  |   FR_4  |   FR_4  |   FR_4  |   FR_8  |   FR_8 |   FR_16|
  |________|_________|_________|_________|_________|_________|_________|________|________|

  if pll clock source selects hext with other frequency values, or configure pll to other
  frequency values, please use the at32 new clock  configuration tool for configuration.  */
  crm_pll_config(CRM_PLL_SOURCE_HEXT, 72, 1, CRM_PLL_FR_2);

  /* enable pll */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);

  /* wait till pll is ready */
  while (crm_flag_get(CRM_PLL_STABLE_FLAG) != SET) {
  }

  /* config ahbclk */
  crm_ahb_div_set(CRM_AHB_DIV_1);

  /* config apb2clk */
  crm_apb2_div_set(CRM_APB2_DIV_2);

  /* config apb1clk */
  crm_apb1_div_set(CRM_APB1_DIV_2);

  /* enable auto step mode */
  crm_auto_step_mode_enable(TRUE);

  /* select pll as system clock source */
  crm_sysclk_switch(CRM_SCLK_PLL);

  /* wait till pll is used as system clock source */
  while (crm_sysclk_switch_status_get() != CRM_SCLK_PLL) {
  }

  /* disable auto step mode */
  crm_auto_step_mode_enable(FALSE);
}

__attribute__((__used__)) void SystemInit(void) {
#if defined(__FPU_USED) && (__FPU_USED == 1U)
  SCB->CPACR |= ((3U << 10U * 2U) | /* set cp10 full access */
                 (3U << 11U * 2U)); /* set cp11 full access */
#endif

  /* reset the crm clock configuration to the default reset state(for debug purpose) */
  /* set hicken bit */
  CRM->ctrl_bit.hicken = TRUE;

  /* wait hick stable */
  while (CRM->ctrl_bit.hickstbl != SET)
    ;

  /* hick used as system clock */
  CRM->cfg_bit.sclksel = CRM_SCLK_HICK;

  /* wait sclk switch status */
  while (CRM->cfg_bit.sclksts != CRM_SCLK_HICK)
    ;

  /* reset hexten, hextbyps, cfden and pllen bits */
  CRM->ctrl &= ~(0x010D0000U);

  /* reset cfg register, include sclk switch, ahbdiv, apb1div, apb2div, adcdiv, clkout bits */
  CRM->cfg = 0;

  /* reset pllms pllns pllfr pllrcs bits */
  CRM->pllcfg = 0x00033002U;

  /* reset clkout[3], usbbufs, hickdiv, clkoutdiv */
  CRM->misc1 = 0;

  /* disable all interrupts enable and clear pending bits  */
  CRM->clkint = 0x009F0000U;

  SCB->VTOR = FLASH_BASE | 0; /* vector table relocation in internal flash. */

  system_clock_config();
}
