#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructurn;
	GPIO_InitStructurn.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructurn.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_2;  // PA3=PWMA-1, PA4=PWMA-2
	GPIO_InitStructurn.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructurn);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision    = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode      = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period           = 100 - 1; // ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler        = 720 - 1; // PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter= 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = 0;                  // CCR
	
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  // PA2 = TIM2_CH3
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  // PA3 = TIM2_CH4
	
	TIM_Cmd(TIM2, ENABLE);
}

void PWM_Set_left(uint16_t compare)
{
	TIM_SetCompare4(TIM2, compare);  // PA3 = TIM2_CH4 = PWMA-1
}

void PWM_Set_right(uint16_t compare)
{
	TIM_SetCompare3(TIM2, compare);  // PA2 = TIM2_CH3 = PWMA-2
}
