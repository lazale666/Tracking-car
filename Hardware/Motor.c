#include "stm32f10x.h"    // Device header
#include "PWM.H"          //PWM头文件用于配置PWM输出波形，此头文件用于配置电机速度及正反转

//PA8     IN1-1 左
//PB13    IN2-2
//PB14    IN1-2 
//PB15    IN2-1 左


void Motor_Init(void)//初始化电机输出引脚
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructurn;
	
	GPIO_InitStructurn.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructurn.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructurn.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructurn);
	
	GPIO_InitStructurn.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructurn);
	
	PWM_Init();
}

void Motor_Set_left(int8_t speed)//PA8(AIN1-1) PB15(AIN2-1)为左轮控制引脚
	                               //PA8—1  PB15—0时正转    PA8—0  PB15—1时反转   
{
	if(speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
		PWM_Set_left(speed);
	}
	if(speed < 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
		PWM_Set_left(-speed);
	}
}

void Motor_Set_right(int8_t speed)//PB14(BIN1-1) PB13(BIN2-1)为右轮控制引脚
	                                //PB14—1  PB13—0时正转    PB14—0  PB13—1时反转  
{
	if(speed >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		PWM_Set_right(speed);
	}
	if(speed < 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_13);
		GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		PWM_Set_right(-speed);
	}
}
