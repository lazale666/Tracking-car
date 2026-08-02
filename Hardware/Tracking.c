#include "stm32f10x.h"                  // Device header

void Tracking_Init(void) //初始化PA9、PA10、PA11、PA12、PB14、PB15作红外检测引脚
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_Initstructurn;
	GPIO_Initstructurn.GPIO_Mode =GPIO_Mode_IPU;
	GPIO_Initstructurn.GPIO_Pin  =GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Initstructurn.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA,&GPIO_Initstructurn);
	
	GPIO_Initstructurn.GPIO_Pin  =GPIO_Pin_14 | GPIO_Pin_15;
	
	GPIO_Init(GPIOB,&GPIO_Initstructurn);
}

uint8_t get_dat(void)//分别为PA9、PA10、PA11、PA12、PB14、PB15赋值
	                   //       -3   -2    -1    +1    +2    +3
{
	uint8_t turn_dat;
	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)==0)turn_dat-=3;
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)==0)turn_dat-=2;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)==0)turn_dat-=1;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9)==0)turn_dat+=1;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)==0)turn_dat+=2;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)==0)turn_dat+=3;
	
	return turn_dat;
}

uint8_t stop_get(void)//停车检测    当PA11  PA12同时收到信号时，表示出现横置黑线（停车标识）
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)==0 && GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9)==0)
		return 1;
	else
		return 0;
}
