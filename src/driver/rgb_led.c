//**********************************************WARNING - THIS FILE NOT YET PORTED FROM F0 SILVERWARE***********************************

#include "driver/rgb_led.h"

#include "core/project.h"
#include "driver/spi.h"
#include "driver/time.h"
#include "util/util.h"

#if (RGB_LED_NUMBER > 0)
void rgb_send(int data);

#ifdef RGB_LED_DMA

#define RGB_BIT_TIME ((SYS_CLOCK_FREQ_HZ / 1000 / 800) - 1)
#define RGB_T0H_TIME (RGB_BIT_TIME * 0.30 + 0.05)
#define RGB_T1H_TIME (RGB_BIT_TIME * 0.60 + 0.05)

extern int rgb_led_value[];

volatile int rgb_dma_phase = 0; // 3:rgb data ready
                                // 2:rgb dma buffer ready

//	 wait to be cascaded after dshot, or fired at next frame if no dshot activity
// 1:rgb dma busy
// 0:idle

const int offset = RGB_PIN > LL_GPIO_PIN_7;

volatile uint32_t rgb_data_portA[RGB_LED_NUMBER * 24 / 4] = {0}; // DMA buffer: reset output when bit data=0 at TOH timing
volatile uint16_t rgb_portX[1] = {RGB_PIN};                      // sum of all rgb pins at port
volatile uint32_t RGB_DATA16[16];                                // 4-bit look-up table for dma buffer making

void rgb_init() {
  if ((RGB_PIN == LL_GPIO_PIN_13 || RGB_PIN == LL_GPIO_PIN_14) && RGB_PORT == GPIOA) {
    // programming port used
    // wait until 2 seconds from powerup passed
    while (time_micros() < 2e6)
      ;
  }

  LL_GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = RGB_PIN;
  LL_GPIO_Init(RGB_PORT, &GPIO_InitStructure);

#ifndef USE_DSHOT_DMA_DRIVER
  // RGB timer/DMA init
  // TIM1_UP  DMA_CH5: set all output to HIGH		at TIM1 update
  // TIM1_CH1 DMA_CH2: reset output if data=0		at T0H timing
  // TIM1_CH4 DMA_CH4: reset all output					at T1H timing

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  // TIM1 Periph clock enable
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = RGB_BIT_TIME;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  TIM_ARRPreloadConfig(TIM1, DISABLE);

  /* Timing Mode configuration: Channel 1 */
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OCInitStructure.CompareValue = RGB_T0H_TIME;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OCInitStructure);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH1);

  /* Timing Mode configuration: Channel 4 */
  TIM_OCInitStructure.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OCInitStructure.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OCInitStructure.CompareValue = RGB_T1H_TIME;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OCInitStructure);
  LL_TIM_OC_DisablePreload(TIM1, LL_TIM_CHANNEL_CH4);

  LL_LL_DMA_InitTypeDef DMA_InitStructure;

  LL_DMA_StructInit(&DMA_InitStructure);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* DMA1 Channe5 configuration ----------------------------------------------*/
  LL_DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_PORT->BSRR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rgb_portX;
  DMA_InitStructure.Direction = DMA_DIR_PeripheralDST;
  DMA_InitStructure.NbData = RGB_LED_NUMBER * 24;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);

  /* DMA1 Channel2 configuration ----------------------------------------------*/
  LL_DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_PORT->BRR + offset;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rgb_data_portA;
  DMA_InitStructure.Direction = DMA_DIR_PeripheralDST;
  DMA_InitStructure.NbData = RGB_LED_NUMBER * 24;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);

  /* DMA1 Channel4 configuration ----------------------------------------------*/
  LL_DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t)&RGB_PORT->BRR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)rgb_portX;
  DMA_InitStructure.Direction = DMA_DIR_PeripheralDST;
  DMA_InitStructure.NbData = RGB_LED_NUMBER * 24;
  DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_NOINCREMENT;
  DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMA_InitStructure.Mode = LL_DMA_MODE_NORMAL;
  DMA_InitStructure.Priority = LL_DMA_PRIORITY_HIGH;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

  TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);

  NVIC_InitTypeDef NVIC_InitStructure;
  /* configure DMA1 Channel4 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = (uint8_t)LL_DMA_PRIORITY_HIGH;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* enable DMA1 Channel4 transfer complete interrupt */
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
#endif

  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    rgb_led_value[i] = 0;
  }
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;

  int pin = rgb_portX[0];

  if (offset) {
    pin >>= 8;
  }

  for (int i = 0; i < 16; i++) {
    RGB_DATA16[i] = (i & 0x01) ? 0 : pin;
    RGB_DATA16[i] <<= 8;
    RGB_DATA16[i] |= (i & 0x02) ? 0 : pin;
    RGB_DATA16[i] <<= 8;
    RGB_DATA16[i] |= (i & 0x04) ? 0 : pin;
    RGB_DATA16[i] <<= 8;
    RGB_DATA16[i] |= (i & 0x08) ? 0 : pin;
  }
}

void rgb_dma_buffer_making() {
  // generate rgb dma packet
  int j = 0;
  for (int n = 0; n < RGB_LED_NUMBER; n++) {
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n] >> 20) & 0x0f];
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n] >> 16) & 0x0f];
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n] >> 12) & 0x0f];
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n] >> 8) & 0x0f];
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n] >> 4) & 0x0f];
    rgb_data_portA[j++] = RGB_DATA16[(rgb_led_value[n]) & 0x0f];
  }
}

void rgb_dma_trigger() {
  TIM1->ARR = RGB_BIT_TIME;
  TIM1->CCR1 = RGB_T0H_TIME;
  TIM1->CCR4 = RGB_T1H_TIME;

  DMA1_Channel5->CPAR = (uint32_t)&RGB_PORT->BSRR;
  DMA1_Channel5->CMAR = (uint32_t)rgb_portX;
  DMA1_Channel2->CPAR = (uint32_t)&RGB_PORT->BRR + offset;
  DMA1_Channel2->CMAR = (uint32_t)rgb_data_portA;
  DMA1_Channel4->CPAR = (uint32_t)&RGB_PORT->BRR;
  DMA1_Channel4->CMAR = (uint32_t)rgb_portX;

  DMA1_Channel2->CCR &= ~(DMA_CCR_MSIZE_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_0 | DMA_CCR_PSIZE_1); // switch from halfword to byte

  DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_GL4 | DMA1_FLAG_GL5);

  DMA1_Channel5->CNDTR = RGB_LED_NUMBER * 24;
  DMA1_Channel2->CNDTR = RGB_LED_NUMBER * 24;
  DMA1_Channel4->CNDTR = RGB_LED_NUMBER * 24;

  TIM1->SR = 0;

  DMA_Cmd(DMA1_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel4, ENABLE);
  DMA_Cmd(DMA1_Channel5, ENABLE);

  TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, ENABLE);

  TIM_SetCounter(TIM1, RGB_BIT_TIME);
  TIM_Cmd(TIM1, ENABLE);
}

void rgb_dma_start() {
  if (rgb_dma_phase <= 1)
    return;

  if (rgb_dma_phase == 3) {
    rgb_dma_buffer_making();
    rgb_dma_phase = 2;
    return;
  }

#ifdef USE_DSHOT_DMA_DRIVER
  extern int dshot_dma_phase;
  if (dshot_dma_phase)
    return;
#endif

  rgb_dma_phase = 1;
  rgb_dma_trigger();
}

void rgb_send(int data) {
  if (!rgb_dma_phase)
    rgb_dma_phase = 3;
}

// if dshot dma is used the routine is in that file
#if !defined(USE_DSHOT_DMA_DRIVER) && defined(RGB_LED_DMA) && (RGB_LED_NUMBER > 0)

void DMA1_Channel4_5_IRQHandler() {
  DMA_Cmd(DMA1_Channel5, DISABLE);
  DMA_Cmd(DMA1_Channel2, DISABLE);
  DMA_Cmd(DMA1_Channel4, DISABLE);

  TIM_DMACmd(TIM1, TIM_DMA_Update | TIM_DMA_CC4 | TIM_DMA_CC1, DISABLE);
  DMA_ClearITPendingBit(DMA1_IT_TC4);
  TIM_Cmd(TIM1, DISABLE);

  rgb_dma_phase = 0;
}
#endif

#else

// sets all leds to a brightness
void rgb_init() {
  // spi port inits

  if ((RGB_PIN == LL_GPIO_PIN_13 || RGB_PIN == LL_GPIO_PIN_14) && RGB_PORT == GPIOA) {
    // programming port used

    // wait until 2 seconds from powerup passed
    while (time_micros() < 2e6)
      ;
  }

  LL_GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin = RGB_PIN;
  LL_GPIO_Init(RGB_PORT, &GPIO_InitStructure);

  for (int i = 0; i < RGB_LED_NUMBER; i++) {
    rgb_send(0);
  }
}

#define RGBHIGH gpioset(RGB_PORT, RGB_PIN)
#define RGBLOW gpioreset(RGB_PORT, RGB_PIN);

#pragma push

#pragma Otime
#pragma O2

void delay1a() {
  uint8_t count = 2;
  while (count--)
    ;
}

void delay1b() {
  uint8_t count = 2;
  while (count--)
    ;
}

void delay2a() {
  uint8_t count = 1;
  while (count--)
    ;
}

void delay2b() {
  uint8_t count = 3;
  while (count--)
    ;
}

void rgb_send(int data) {
  for (int i = 23; i >= 0; i--) {
    if ((data >> i) & 1) {
      RGBHIGH;
      delay1a();
      RGBLOW;
      delay1b();
    } else {
      RGBHIGH;
      delay2a();
      RGBLOW;
      delay2b();
    }
  }
}
#pragma pop

#endif

#else
// rgb led not found
// some dummy headers just in case
void rgb_init() {
}

void rgb_send(int data) {
}
#endif
