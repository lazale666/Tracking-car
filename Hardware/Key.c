#include "stm32f10x.h"                  // Device header

void Key_Init(void)//按键初始化
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//打开按键时钟
	//配置PB0，PB1引脚
	GPIO_InitTypeDef GPIO_Initstructurn;
	GPIO_Initstructurn.GPIO_Mode =GPIO_Mode_IPU;
	GPIO_Initstructurn.GPIO_Pin  =GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5;
	GPIO_Initstructurn.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOB,&GPIO_Initstructurn);
}
//设置键码值  key—1：PB0     key—2：PB1          测量按键
uint8_t Key_Num(void)
{
	uint8_t i=0;
	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==0)i=1;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)==0)i=2;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==0)i=3;
	
	return i;
}
