#include "stm32f10x.h"    // Device header
#include "PWM.H"          //PWM头文件用于配置PWM输出波形，此头文件用于配置电机速度及正反转

void Motor_Init(void)//初始化电机输出引脚
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructurn;
	
	GPIO_InitStructurn.GPIO_Mode  =GPIO_Mode_Out_PP;
	GPIO_InitStructurn.GPIO_Pin   =GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructurn.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructurn);
	
	PWM_Init();
}

void Motor_Set_left(int8_t speed)//PA4 PA5为左轮控制引脚
	                               //PA4—1  PA5—0时正转    PA4—0  PA5—1时反转   
{
	if(speed>=0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		PWM_Set_left(speed);
	}
	if(speed<0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		PWM_Set_left(-speed);
	}
}

void Motor_Set_right(int8_t speed)//PA6 PA7为左轮控制引脚
	                                //PA6—1  PA7—0时正转    PA6—0  PA7—1时反转  
{
	if(speed>=0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
		PWM_Set_right(speed);
	}
	if(speed<0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_7);
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
		PWM_Set_right(-speed);
	}
}
