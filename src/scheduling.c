#include "lab04_tasks.h"
#include "scheduling.h"

uint16_t CCR1_Val = 0;
uint16_t CCR2_Val = 0;
uint16_t CCR3_Val = 0;
uint16_t CCR4_Val = 0;

uint16_t Period = 2000;
uint16_t PrescalerValue = 0;
uint16_t counter = 0;
MotorSpeeds newspeeds = {.m1 = 0, .m2 = 0, .m3 = 0, .m4 = 0};

int motorInit(void)
{

  RCC_Configuration();

  GPIO_Configuration();

  PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;

  TIM_TimeBaseStructure.TIM_Period = Period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val; 

  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;

  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_Pulse = CCR3_Val; 

  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM4, ENABLE); 

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

}

void ChangeMotorSpeed(MotorSpeeds* p_motorSpeedsPtr)
{
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m1 * 100;
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m2 * 100;
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m3 * 100;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = p_motorSpeedsPtr->m4 * 100;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure); 

}


void RCC_Configuration(void)
{
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void LEDInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; // stm32f10x_gpio.h
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED_GPIO_CLK, ENABLE); // include stm32f10x_rcc.c, chang GPIO_CLK[LED] to RCC_APB2Periph_GPIOB

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_GPIO_PORT, &GPIO_InitStructure); //stm32f10x_gpio.c
}

void LEDToggle(void)
{
  LED_GPIO_PORT->ODR ^= LED_PIN;
}

void LEDOn(void)
{
  LED_GPIO_PORT->BRR = LED_PIN; 
}

void LEDOff(void)
{
  LED_GPIO_PORT->BSRR = LED_PIN; 
}

void EnableTimerInterrupt(void)
{
    // TIM2 Enable
	  NVIC_InitTypeDef nvicStructure_2;
    nvicStructure_2.NVIC_IRQChannel = TIM2_IRQn;
    nvicStructure_2.NVIC_IRQChannelPreemptionPriority = 0;
    nvicStructure_2.NVIC_IRQChannelSubPriority = 0;
    nvicStructure_2.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure_2);

    // TIM1 Enable
    NVIC_InitTypeDef nvicStructure_1;
    nvicStructure_1.NVIC_IRQChannel = TIM1_UP_IRQn;
    nvicStructure_1.NVIC_IRQChannelPreemptionPriority = 1;
    nvicStructure_1.NVIC_IRQChannelSubPriority = 0;
    nvicStructure_1.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure_1);
}

void TM_Init(void)
{

	PrescalerValue = (uint16_t) (SystemCoreClock / 2000) - 1;
  // TIM2 with Frequency @1Hz
 	TIM_TimeBaseStructure2.TIM_Period = Period;
  TIM_TimeBaseStructure2.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure2.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure2.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure2.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure2);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	// EnableTimerInterrupt();

  // TIM1 with Frequency @100Hz
  TIM_TimeBaseStructure2.TIM_Prescaler = (uint16_t) (SystemCoreClock / 200000) - 1;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure2);
  TIM_Cmd(TIM1, ENABLE);
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

  EnableTimerInterrupt();
}

void TIM2_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	ms.m1 = 1;
	ms.m2 = 1;
	ms.m3 = 1;
	ms.m4 = 1;
	calculateOrientation();
  updatePid(&newspeeds);
  ChangeMotorSpeed(&newspeeds);
}

void TIM1_UP_IRQHandler(void)
{
  TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
  ms.m1 = 1;
  ms.m2 = 1;
  ms.m3 = 1;
  ms.m4 = 1;
  
  detectEmergency();

  if (counter == 10) {
    refreshSensorData();
    LEDToggle();
    counter = 0;
  }
  else
    counter ++;
}

void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}