__attribute__((weak)) void Default_Handler();

__attribute__((used)) void Default_Handler_Proxy() {
  Default_Handler();
}

__attribute__((weak, alias("Default_Handler_Proxy"))) void NMI_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void HardFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void MemManage_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void BusFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void UsageFault_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SVC_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DebugMon_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void PendSV_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SysTick_Handler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void WWDG_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void PVD_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TAMP_STAMP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RTC_WKUP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void FLASH_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RCC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void ADC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI9_5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_BRK_TIM9_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_UP_TIM10_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_TRG_COM_TIM11_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM1_CC_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C1_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C1_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C2_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C2_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void EXTI15_10_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void RTC_Alarm_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_FS_WKUP_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA1_Stream7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SDIO_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void TIM5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream0_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream1_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream2_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream3_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void OTG_FS_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream5_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void DMA2_Stream7_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void USART6_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C3_EV_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void I2C3_ER_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void FPU_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI4_IRQHandler();
__attribute__((weak, alias("Default_Handler_Proxy"))) void SPI5_IRQHandler();
