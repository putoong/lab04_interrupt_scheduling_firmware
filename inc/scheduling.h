#ifndef __SCHEDULING_H
#define __SCHEDULING_H

#include "lab04_tasks.h"
#include "sys_clk_init.h"

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure2;

#define LED_PIN_RED                     GPIO_Pin_4
#define LED_PIN_GREEN                   GPIO_Pin_5
#define LED_GPIO_PORT                   GPIOB
#define LED_GPIO_CLK                    RCC_APB2Periph_GPIOB  

#define SPEED 							100


void RCC_Configuration(void);
void GPIO_Configuration(void);
void ChangeMotorSpeed(MotorSpeeds* p_motorSpeedsPtr);
void TM_Init(void);
void TIM2_IRQHandler(void);
void EnableTimerInterrupt(void);
void LEDInit(void);
void LEDToggle_RED(void);
void LEDToggle_GREEN(void);
void LEDOn(void);
void LEDOff(void);

#endif