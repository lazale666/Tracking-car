#include "stm32f10x.h"                  // Device header

void Key_Init(void)//按键初始化
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//打开按键时钟
	//配置PA0，PA1，PA2，PA12引脚
	GPIO_InitTypeDef GPIO_Initstructurn;
	GPIO_Initstructurn.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Initstructurn.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_12;
	GPIO_Initstructurn.GPIO_Speed= GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &GPIO_Initstructurn);
}
//设置键码值  key—1：PA0     key—2：PA1     key—3：PA4     key—4：PA12
uint8_t Key_Num(void)
{
	uint8_t i = 0;
	
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0) i = 1;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0) i = 2;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0) i = 3;
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0) i = 4;
	
	return i;
}
