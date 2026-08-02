#include "stm32f10x.h"                  // Device header
#include "Delay.H"

void Buzzer_Init(void)//蜂鸣器初始化
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//配置PB12引脚
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void buzzer_set(uint8_t setdata)
{
	if(setdata)
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	else
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}
