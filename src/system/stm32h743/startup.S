/**
  ******************************************************************************
  * @file      startup_stm32h743xx.s
  * @author    MCD Application Team
  * @brief     STM32H743xx Devices vector table for GCC based toolchain. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
    
  .syntax unified
  .cpu cortex-m7
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
  .weak  Reset_Handler
  .type  Reset_Handler, %function
Reset_Handler:  
  ldr   sp, =_estack      /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

  bl system_check_for_bootloader
/* Call the clock system initialization function.*/
  bl  SystemInit   
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl  memory_section_init
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler

/******************************************************************************
*
* The minimal vector table for a Cortex M. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
* 
*******************************************************************************/
   .section  .isr_vector,"a",%progbits
  .type  g_pfnVectors, %object
  .size  g_pfnVectors, .-g_pfnVectors
   
   
g_pfnVectors:
  .word  _estack
  .word  Reset_Handler

  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler
  
  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */                                        
  .word     PVD_AVD_IRQHandler                /* PVD/AVD through EXTI Line detection */                        
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */            
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */                      
  .word     FLASH_IRQHandler                  /* FLASH                        */                                          
  .word     RCC_IRQHandler                    /* RCC                          */                                            
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */                        
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */                          
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */                          
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */                          
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */                          
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */                  
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */                   
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */                   
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */                   
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */                   
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */                   
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */                   
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */                   
  .word     FDCAN1_IT0_IRQHandler             /* FDCAN1 interrupt line 0      */                    
  .word     FDCAN2_IT0_IRQHandler             /* FDCAN2 interrupt line 0      */                    
  .word     FDCAN1_IT1_IRQHandler             /* FDCAN1 interrupt line 1      */                    
  .word     FDCAN2_IT1_IRQHandler             /* FDCAN2 interrupt line 1      */                    
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */                          
  .word     TIM1_BRK_IRQHandler               /* TIM1 Break interrupt         */         
  .word     TIM1_UP_IRQHandler                /* TIM1 Update interrupt        */         
  .word     TIM1_TRG_COM_IRQHandler           /* TIM1 Trigger and Commutation interrupt */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */                          
  .word     TIM2_IRQHandler                   /* TIM2                         */                   
  .word     TIM3_IRQHandler                   /* TIM3                         */                   
  .word     TIM4_IRQHandler                   /* TIM4                         */                   
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */                          
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */                          
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */                          
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */                            
  .word     SPI1_IRQHandler                   /* SPI1                         */                   
  .word     SPI2_IRQHandler                   /* SPI2                         */                   
  .word     USART1_IRQHandler                 /* USART1                       */                   
  .word     USART2_IRQHandler                 /* USART2                       */                   
  .word     USART3_IRQHandler                 /* USART3                       */                   
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */                          
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */                 
  .word     0                                 /* Reserved                     */                       
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */         
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */         
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */                          
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */                          
  .word     FMC_IRQHandler                    /* FMC                          */                   
  .word     SDMMC1_IRQHandler                 /* SDMMC1                       */                   
  .word     TIM5_IRQHandler                   /* TIM5                         */                   
  .word     SPI3_IRQHandler                   /* SPI3                         */                   
  .word     UART4_IRQHandler                  /* UART4                        */                   
  .word     UART5_IRQHandler                  /* UART5                        */                   
  .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */                   
  .word     TIM7_IRQHandler                   /* TIM7                         */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */                   
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */                   
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */                   
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */                   
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */                   
  .word     ETH_IRQHandler                    /* Ethernet                     */                   
  .word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */                     
  .word     FDCAN_CAL_IRQHandler              /* FDCAN calibration unit interrupt*/                                               
  .word     0                                 /* Reserved                     */                          
  .word     0                                 /* Reserved                     */                       
  .word     0                                 /* Reserved                     */                           
  .word     0                                 /* Reserved                     */                  
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */                   
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */                   
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */                   
  .word     USART6_IRQHandler                 /* USART6                       */                    
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */                          
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */                          
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */                   
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */                   
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */                         
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */                   
  .word     DCMI_IRQHandler                   /* DCMI                         */                   
  .word     0                                 /* Reserved                     */                   
  .word     RNG_IRQHandler                    /* Rng                          */
  .word     FPU_IRQHandler                    /* FPU                          */
  .word     UART7_IRQHandler                  /* UART7                        */      
  .word     UART8_IRQHandler                  /* UART8                        */
  .word     SPI4_IRQHandler                   /* SPI4                         */
  .word     SPI5_IRQHandler                   /* SPI5                         */
  .word     SPI6_IRQHandler                   /* SPI6                         */
  .word     SAI1_IRQHandler                   /* SAI1                         */
  .word     LTDC_IRQHandler                   /* LTDC                         */
  .word     LTDC_ER_IRQHandler                /* LTDC error                   */
  .word     DMA2D_IRQHandler                  /* DMA2D                        */
  .word     SAI2_IRQHandler                   /* SAI2                         */
  .word     QUADSPI_IRQHandler                /* QUADSPI                      */
  .word     LPTIM1_IRQHandler                 /* LPTIM1                       */
  .word     CEC_IRQHandler                    /* HDMI_CEC                     */
  .word     I2C4_EV_IRQHandler                /* I2C4 Event                   */
  .word     I2C4_ER_IRQHandler                /* I2C4 Error                   */
  .word     SPDIF_RX_IRQHandler               /* SPDIF_RX                     */ 
  .word     OTG_FS_EP1_OUT_IRQHandler         /* USB OTG FS End Point 1 Out   */   
  .word     OTG_FS_EP1_IN_IRQHandler          /* USB OTG FS End Point 1 In    */   
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI */  
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  .word     DMAMUX1_OVR_IRQHandler            /* DMAMUX1 Overrun interrupt    */  
  .word     HRTIM1_Master_IRQHandler          /* HRTIM Master Timer global Interrupt */
  .word     HRTIM1_TIMA_IRQHandler            /* HRTIM Timer A global Interrupt */  
  .word     HRTIM1_TIMB_IRQHandler            /* HRTIM Timer B global Interrupt */  
  .word     HRTIM1_TIMC_IRQHandler            /* HRTIM Timer C global Interrupt */  
  .word     HRTIM1_TIMD_IRQHandler            /* HRTIM Timer D global Interrupt */  
  .word     HRTIM1_TIME_IRQHandler            /* HRTIM Timer E global Interrupt */  
  .word     HRTIM1_FLT_IRQHandler             /* HRTIM Fault global Interrupt   */ 
  .word     DFSDM1_FLT0_IRQHandler            /* DFSDM Filter0 Interrupt        */
  .word     DFSDM1_FLT1_IRQHandler            /* DFSDM Filter1 Interrupt        */     
  .word     DFSDM1_FLT2_IRQHandler            /* DFSDM Filter2 Interrupt        */    
  .word     DFSDM1_FLT3_IRQHandler            /* DFSDM Filter3 Interrupt        */   
  .word     SAI3_IRQHandler                   /* SAI3 global Interrupt          */   
  .word     SWPMI1_IRQHandler                 /* Serial Wire Interface 1 global interrupt */
  .word     TIM15_IRQHandler                  /* TIM15 global Interrupt      */        
  .word     TIM16_IRQHandler                  /* TIM16 global Interrupt      */      
  .word     TIM17_IRQHandler                  /* TIM17 global Interrupt      */      
  .word     MDIOS_WKUP_IRQHandler             /* MDIOS Wakeup  Interrupt     */     
  .word     MDIOS_IRQHandler                  /* MDIOS global Interrupt      */    
  .word     JPEG_IRQHandler                   /* JPEG global Interrupt       */    
  .word     MDMA_IRQHandler                   /* MDMA global Interrupt       */    
  .word     0                                 /* Reserved                    */    
  .word     SDMMC2_IRQHandler                 /* SDMMC2 global Interrupt     */    
  .word     HSEM1_IRQHandler                  /* HSEM1 global Interrupt      */    
  .word     0                                 /* Reserved                    */    
  .word     ADC3_IRQHandler                   /* ADC3 global Interrupt       */    
  .word     DMAMUX2_OVR_IRQHandler            /* DMAMUX Overrun interrupt    */    
  .word     BDMA_Channel0_IRQHandler          /* BDMA Channel 0 global Interrupt */
  .word     BDMA_Channel1_IRQHandler          /* BDMA Channel 1 global Interrupt */ 
  .word     BDMA_Channel2_IRQHandler          /* BDMA Channel 2 global Interrupt */ 
  .word     BDMA_Channel3_IRQHandler          /* BDMA Channel 3 global Interrupt */ 
  .word     BDMA_Channel4_IRQHandler          /* BDMA Channel 4 global Interrupt */ 
  .word     BDMA_Channel5_IRQHandler          /* BDMA Channel 5 global Interrupt */ 
  .word     BDMA_Channel6_IRQHandler          /* BDMA Channel 6 global Interrupt */ 
  .word     BDMA_Channel7_IRQHandler          /* BDMA Channel 7 global Interrupt */ 
  .word     COMP1_IRQHandler                  /* COMP1 global Interrupt     */          
  .word     LPTIM2_IRQHandler                 /* LP TIM2 global interrupt   */      
  .word     LPTIM3_IRQHandler                 /* LP TIM3 global interrupt   */     
  .word     LPTIM4_IRQHandler                 /* LP TIM4 global interrupt   */     
  .word     LPTIM5_IRQHandler                 /* LP TIM5 global interrupt   */     
  .word     LPUART1_IRQHandler                /* LP UART1 interrupt         */     
  .word     0                                 /* Reserved                   */     
  .word     CRS_IRQHandler                    /* Clock Recovery Global Interrupt */ 
  .word     ECC_IRQHandler                    /* ECC diagnostic Global Interrupt */ 
  .word     SAI4_IRQHandler                   /* SAI4 global interrupt      */      
  .word     0                                 /* Reserved                   */      
  .word     0                                 /* Reserved                   */      
  .word     WAKEUP_PIN_IRQHandler             /* Interrupt for all 6 wake-up pins */
   
 