/**
  ******************************************************************************
  * @file    stm32f0xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   This file contains all the functions prototypes for the
  *          stm32f0xx_nucleo.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F0XX_NUCLEO_H
#define __STM32F0XX_NUCLEO_H

#ifdef __cplusplus
 extern "C" {
#endif
                                              
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
   
/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32F0XX_NUCLEO
  * @{
  */
      
/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL 
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32NUCLEO Kit from STMicroelectronics.
  * @{
  */

/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL_Exported_Types
  * @{
  */
typedef enum 
{
  LED2 = 0,
  
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

/* The Joystick is available on adafruit 1.8" TFT shield */     
typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef;

/**
  * @}
  */

/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL_Exported_Macros
  * @{
  */
/**
  * @brief LCD control lines (CS and DC) management
  */
#define LCD_CS_HIGH() LCD_CS_GPIO_PORT->BSRR = LCD_CS_PIN
#define LCD_CS_LOW()  LCD_CS_GPIO_PORT->BRR = LCD_CS_PIN
#define LCD_DC_HIGH() LCD_DC_GPIO_PORT->BSRR = LCD_DC_PIN
#define LCD_DC_LOW()  LCD_DC_GPIO_PORT->BRR = LCD_DC_PIN

/**
  * @brief SD CS line management 
  */
#define SD_CS_HIGH()  SD_CS_GPIO_PORT->BSRR = SD_CS_PIN
#define SD_CS_LOW()   SD_CS_GPIO_PORT->BRR = SD_CS_PIN    

/**
  * @}
  */  

/** @defgroup STM32F0xx_NUCLEO_LOW_LEVEL_Exported_Constants
  * @{
  */ 
/** @addtogroup STM32F0xx_NUCLEO_LOW_LEVEL_LED
  * @{
  */
#define LEDn                             1

#define LED2_PIN                         GPIO_Pin_5
#define LED2_GPIO_PORT                   GPIOA
#define LED2_GPIO_CLK                    RCC_AHBPeriph_GPIOA  

/**
  * @}
  */ 
/** @addtogroup STM32F0xx_NUCLEO_LOW_LEVEL_BUTTON
  * @{
  */  
#define BUTTONn                          1  

/**
 * @brief Wakeup push-button
 */
#define USER_BUTTON_PIN                GPIO_Pin_13
#define USER_BUTTON_GPIO_PORT          GPIOC
#define USER_BUTTON_GPIO_CLK           RCC_AHBPeriph_GPIOC
#define USER_BUTTON_EXTI_LINE          EXTI_Line13
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOC
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource13
#define USER_BUTTON_EXTI_IRQn          EXTI4_15_IRQn 
/**
  * @}
  */
    
/** @addtogroup STM32F0XX_NUCLEO_LOW_LEVEL_SPI
  * @{
  */
/**
  * @brief  SPI Interface pins
  */
/* The LCD and uSD card are available on adafruit 1.8" TFT shield */
#define LCD_SD_SPI                    SPI1
#define LCD_SD_SPI_CLK                RCC_APB2Periph_SPI1

#define SPI_SCK_PIN                   GPIO_Pin_5                  /* PA.05 */
#define SPI_SCK_GPIO_PORT             GPIOA                       /* GPIOA */
#define SPI_SCK_GPIO_CLK              RCC_AHBPeriph_GPIOA
#define SPI_SCK_SOURCE                GPIO_PinSource5
#define SPI_SCK_AF                    GPIO_AF_0

#define SPI_MISO_PIN                  GPIO_Pin_6                  /* PA.06 */
#define SPI_MISO_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_MISO_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SPI_MISO_SOURCE               GPIO_PinSource6
#define SPI_MISO_AF                   GPIO_AF_0

#define SPI_MOSI_PIN                  GPIO_Pin_7                  /* PA.07 */
#define SPI_MOSI_GPIO_PORT            GPIOA                       /* GPIOA */
#define SPI_MOSI_GPIO_CLK             RCC_AHBPeriph_GPIOA
#define SPI_MOSI_SOURCE               GPIO_PinSource7
#define SPI_MOSI_AF                   GPIO_AF_0
/**
  * @}
  */  
    
/**
  * @brief  SD Chip Select pin
  */
#define SD_CS_PIN                     GPIO_Pin_5                  /* PB.5 */
#define SD_CS_GPIO_PORT               GPIOB                       /* GPIOB */
#define SD_CS_GPIO_CLK                RCC_AHBPeriph_GPIOB

/**
  * @brief  LCD Data/Command pin
  */
#define LCD_DC_PIN                    GPIO_Pin_9                  /* PA.9 */
#define LCD_DC_GPIO_PORT              GPIOA                       /* GPIOA */
#define LCD_DC_GPIO_CLK               RCC_AHBPeriph_GPIOA

/**
  * @brief  LCD Chip Select pin
  */
#define LCD_CS_PIN                    GPIO_Pin_6                  /* PB.6*/
#define LCD_CS_GPIO_PORT              GPIOB                       /* GPIOB */
#define LCD_CS_GPIO_CLK               RCC_AHBPeriph_GPIOB
     
/**
  * @brief  ADC Interface pins
  */
/* The ADC is used to detect motion of Joystick available on adafruit 1.8" TFT shield */   
#define ADCx                            ADC1
#define ADC_CLK                         RCC_APB2Periph_ADC1
    
#define ADC_GPIO_PIN                    GPIO_Pin_0                  /* PB.0  */
#define ADC_GPIO_PORT                   GPIOB                       /* GPIOB */
#define ADC_GPIO_CLK                    RCC_AHBPeriph_GPIOB
/**
  * @}
  */

/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL_Exported_Functions
  * @{
  */
void STM_EVAL_LEDInit(Led_TypeDef Led);
void STM_EVAL_LEDOn(Led_TypeDef Led);
void STM_EVAL_LEDOff(Led_TypeDef Led);
void STM_EVAL_LEDToggle(Led_TypeDef Led);
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button);

/* Functions to configure and manage the STM32F0xx resources (SPI and ADC) used to drive 
   LCD, uSD card and Joystick available in adafruit 1.8" TFT shield */
void STM_SPI_Init(void);
uint8_t STM_SPI_WriteRead(uint8_t Data);
void LCD_CtrlLines_Config(void);
void SD_CtrlLines_Config(void);
void STM_ADC_Config(void);
JOYState_TypeDef STM_Get_JOYState(void);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F0XX_NUCLEO_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
