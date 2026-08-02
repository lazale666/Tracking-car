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

void Buzzer_open(void)//打开蜂鸣器  发出连续两下滴滴的声音信号
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(700);
}

void Buzzer_open_low(void)//打开蜂鸣器  发出连续两下滴滴的声音信号（简短版  用于定时器启动）
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	Delay_ms(100);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
}

void Buzzer_open_key(uint8_t key_command)//打开蜂鸣器  发出短暂一下滴的声音信号（用于按键）
{
	if(key_command == 0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	}
}
