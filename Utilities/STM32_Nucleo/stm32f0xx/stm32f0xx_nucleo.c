/**
  ******************************************************************************
  * @file    stm32f0xx_nucleo.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   This file provides set of firmware functions to manage Leds and
  *          push-button available on STM32NUCLEO Kit from STMicroelectronics.
  *          It provides also functions to configure and manage the STM32F3xx 
  *          resources (SPI and ADC) used to drive LCD, uSD card and Joystick
  *          available on adafruit 1.8" TFT shield.  
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
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_nucleo.h"

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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
    
/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL_Private_Variables
  * @{
  */ 
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED2_PIN};
const uint32_t GPIO_CLK[LEDn] = {LED2_GPIO_CLK};

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {USER_BUTTON_GPIO_PORT}; 

const uint16_t BUTTON_PIN[BUTTONn] = {USER_BUTTON_PIN}; 

const uint32_t BUTTON_CLK[BUTTONn] = {USER_BUTTON_GPIO_CLK};

const uint16_t BUTTON_EXTI_LINE[BUTTONn] = {USER_BUTTON_EXTI_LINE};

const uint8_t BUTTON_PORT_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PORT_SOURCE};
								 
const uint8_t BUTTON_PIN_SOURCE[BUTTONn] = {USER_BUTTON_EXTI_PIN_SOURCE}; 
const uint8_t BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn};

/**
  * @}
  */ 

/* Private function prototypes -----------------------------------------------*/

/** @defgroup STM32F0XX_NUCLEO_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *         This parameter must be: LED2
  * @retval None
  */
void STM_EVAL_LEDInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(GPIO_CLK[Led], ENABLE);
  
  /* Configure the GPIO pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *         This parameter must be: LED2
  * @retval None
  */
void STM_EVAL_LEDOn(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BSRR = GPIO_PIN[Led];      
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *         This parameter must be: LED2
  * @retval None
  */
void STM_EVAL_LEDOff(Led_TypeDef Led)
{
  GPIO_PORT[Led]->BRR = GPIO_PIN[Led];   
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *         This parameter must be: LED2
  * @retval None
  */
void STM_EVAL_LEDToggle(Led_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}
/**
  * @brief  Configures Button GPIO and associated EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter must be: BUTTON_USER
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                            generation capability  
  * @retval None
  */
void STM_EVAL_PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the GPIO Clock */
  RCC_AHBPeriphClockCmd(BUTTON_CLK[Button], ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure Button pin as input */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);

  if (Button_Mode == BUTTON_MODE_EXTI)
  {
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(BUTTON_PORT_SOURCE[Button], BUTTON_PIN_SOURCE[Button]);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_IRQn[Button];
    NVIC_InitStructure.NVIC_IRQChannelPriority= 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 
  }
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter must be: BUTTON_USER  
  * @retval The Button GPIO pin value.
  */
uint32_t STM_EVAL_PBGetState(Button_TypeDef Button)
{
  return GPIO_ReadInputDataBit(BUTTON_PORT[Button], BUTTON_PIN[Button]);
}

/**
  * @brief  Initializes the SPI Interface used to drive the LCD and uSD card 
  *         available on adafruit 1.8" TFT shield.
  * @note   This function must by called by the application code before to initialize
  *         the LCD and uSD card.    
  * @param  None
  * @retval None
  */
void STM_SPI_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef   SPI_InitStructure;

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(SPI_MOSI_GPIO_CLK | SPI_MISO_GPIO_CLK | SPI_SCK_GPIO_CLK, ENABLE);

  /* Enable SPI clock */
  RCC_APB2PeriphClockCmd(LCD_SD_SPI_CLK, ENABLE); 

  /* Configure SPI SCK pin */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure SPI MISO pin */
  GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
  GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

  /* Configure SPI MOSI pin */
  GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect SCK, MISO and MOSI pins to SPI alternate */
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF); 
  
  /* Configure SPI */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
      /* SPI baudrate is set to 9 MHz maximum (PCLK1/SPI_BaudRatePrescaler = 36/4 = 9 MHz) 
       to verify these constraints:
          - ST7735R LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint 
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK1 max frequency is 36 MHz 
       */
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(LCD_SD_SPI, &SPI_InitStructure);
  
  /* Configure RXFIFO to return its state each Quarter Full buffer (8its) */
  SPI_RxFIFOThresholdConfig(LCD_SD_SPI, SPI_RxFIFOThreshold_QF);
  
  /* Enable SPI */
  SPI_Cmd(LCD_SD_SPI, ENABLE);
}

/**
  * @brief  Configures LCD control lines (CS and DC) in Output Push-Pull mode.
  * @param  None
  * @retval None
  */
void LCD_CtrlLines_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIOs clock */
  RCC_AHBPeriphClockCmd(LCD_CS_GPIO_CLK | LCD_DC_GPIO_CLK, ENABLE);

  /* Configure CS in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure DC in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_DC_PIN;
  GPIO_Init(LCD_DC_GPIO_PORT, &GPIO_InitStructure);

  /* Set chip select pin high */
  LCD_CS_HIGH();
}

/**
  * @brief  Configures uSD control line (CS) in Output Push-Pull mode.
  * @param  None
  * @retval None
  */
void SD_CtrlLines_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(SD_CS_GPIO_CLK , ENABLE);

  /* Configure CS in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = SD_CS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Set chip select pin high */
  SD_CS_HIGH();
}

/**
  * @brief  Sends a byte through the SPI interface and return the byte received 
  *         from the SPI bus.
  * @param  Data: byte send.
  * @retval The received byte value
  * @retval None
  */
uint8_t STM_SPI_WriteRead(uint8_t Data)
{
  uint8_t tmp = 0x00;
  
  /* Wait until the transmit buffer is empty */ 
  while(SPI_I2S_GetFlagStatus(LCD_SD_SPI, SPI_I2S_FLAG_TXE) != SET)
  {
  }  
  /* Send the byte */
  SPI_SendData8(LCD_SD_SPI, Data);
  
  /* Wait to receive a byte */ 
  while(SPI_I2S_GetFlagStatus(LCD_SD_SPI, SPI_I2S_FLAG_RXNE) != SET)
  {
  }
  /* Return the byte read from the SPI bus */    
  tmp = SPI_ReceiveData8(LCD_SD_SPI); 

  /* Wait until the BSY flag is set */   
  while(SPI_I2S_GetFlagStatus(LCD_SD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }  
 
  /* Return read Data */
  return tmp;
}

/**
  * @brief  Initializes ADC, used to detect motion of Joystick available on 
  *         adafruit 1.8" TFT shield.
  * @param  None
  * @retval None
  */
void STM_ADC_Config(void)
{
  GPIO_InitTypeDef      GPIO_InitStructure;
  ADC_InitTypeDef       ADC_InitStructure;
      
  /* Enable ADC clock */
  RCC_APB2PeriphClockCmd(ADC_CLK, ENABLE);

  /* Enable GPIO clock */
  RCC_AHBPeriphClockCmd(ADC_GPIO_CLK, ENABLE);
      
  /* Configure ADC1 Channel 8 as analog input */
  GPIO_InitStructure.GPIO_Pin = ADC_GPIO_PIN ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(ADC_GPIO_PORT, &GPIO_InitStructure);  
  ADC_StructInit(&ADC_InitStructure);
  
  /* ADC1 DeInit */  
  ADC_DeInit(ADC1);
  
  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);
  
  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure); 
  
  /* Convert the ADC1 Channel 8 with 239.5 Cycles as sampling time */ 
  ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_239_5Cycles);   

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);
  
  /* Enable ADCperipheral[PerIdx] */
  ADC_Cmd(ADC1, ENABLE);     
  
  /* Wait the ADRDY falg */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
  
  /* ADC1 regular Software Start Conv */ 
  ADC_StartOfConversion(ADC1);

}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - SEL   : 1.055 V / 1308
  *           - RIGHT : 0.595 V / 737
  *           - LEFT  : 3.0 V / 3720 
  *           - UP    : 1.65 V / 2046
  *           - DOWN  : 0.71 V / 88
  *           - None  : 3.3 V / 4095
  * @retval Code of the Joystick key pressed.
  *          This code can be one of the following values:
  *            @arg  JOY_NONE
  *            @arg  JOY_SEL
  *            @arg  JOY_DOWN
  *            @arg  JOY_LEFT
  *            @arg  JOY_RIGHT
  *            @arg  JOY_UP  
  */
JOYState_TypeDef STM_Get_JOYState(void)
{
  JOYState_TypeDef state = JOY_NONE;
  uint16_t  KeyConvertedValue = 0; 
  KeyConvertedValue = ADC_GetConversionValue(ADCx);
  
  if((KeyConvertedValue > 2010) && (KeyConvertedValue < 2090))
  {
    state = JOY_UP;
  }
  else if((KeyConvertedValue > 680) && (KeyConvertedValue < 780))
  {
    state = JOY_RIGHT;
  }
  else if((KeyConvertedValue > 1270) && (KeyConvertedValue < 1350))
  {
    state = JOY_SEL;
  }
  else if((KeyConvertedValue > 50) && (KeyConvertedValue < 130))
  {
    state = JOY_DOWN;
  }
  else if((KeyConvertedValue > 3680) && (KeyConvertedValue < 3760))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }
  /* Loop while a key is pressed */
  if(state != JOY_NONE)
  { 
    while(KeyConvertedValue < 4000)
    {
      KeyConvertedValue = ADC_GetConversionValue(ADCx);
    }      
  }
  /* Return the code of the Joystick key pressed*/
  return state;
}

/**
  * @}
  */ 

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
