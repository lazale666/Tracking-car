#include "stm32f10x.h"                  // Device header

void Timer_Init(void)//配置TIM3用于代替Delay作延时
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//打开TIM3时钟
	//初始化设置TIM3
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision    =TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode      =TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period           =10 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler        =7200 - 1;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除标志位，防止初始化导致输出更新
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	//初始化设置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel                  =TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd               =ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority       =1;
	
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3,ENABLE);
}
