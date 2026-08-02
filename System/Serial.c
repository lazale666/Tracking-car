#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define K230_HEAD 0xAA
#define K230_TAIL 0x55

uint8_t Serial_TxPacket[6];
uint8_t Serial_RxPacket[6];

uint8_t Serial_RxFlag = 0;

int16_t K230_Mod = 0;
int16_t K230_dx = 0;
int16_t K230_Speed = 0;

uint8_t K230_RxBuffer[6];
uint8_t K230_RxCount = 0;

void Serial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  // PB10 = USART3_TX (CAM_TX)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  // PB11 = USART3_RX (CAM_RX)
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

void GetData(int16_t *Mode, int16_t *dx, int16_t *Speed)
{
	K230_Mod = K230_RxBuffer[1];
	
	uint8_t sign_byte = K230_RxBuffer[2];
	uint8_t value_byte = K230_RxBuffer[3];
	
	// 处理dx格式：第一字节00=负(左偏)，01=正(右偏)，第二字节为数值0-100
	if(sign_byte == 0x00)
	{
		K230_dx = -value_byte;  // 左偏为负
	}
	else if(sign_byte == 0x01)
	{
		K230_dx = value_byte;  // 右偏为正
	}
	else
	{
		K230_dx = 0;
	}
	
	K230_Speed = K230_RxBuffer[4];
	
	*Mode = K230_Mod;
	*dx = K230_dx;
	*Speed = K230_Speed;
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void Serial_SendPacket(void)
{
	Serial_SendByte(K230_HEAD);
	for(uint8_t i = 0; i < 4; i++)
	{
		Serial_SendByte(Serial_TxPacket[i]);
	}
	Serial_SendByte(K230_TAIL);
}

void USART3_IRQHandler(void)
{
	uint8_t rx_data;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != SET)
	{
		return;
	}
	
	rx_data = USART_ReceiveData(USART3);
	
	if(K230_RxCount == 0)
	{
		if(rx_data == K230_HEAD)
		{
			K230_RxBuffer[K230_RxCount++] = rx_data;
		}
	}
	else
	{
		K230_RxBuffer[K230_RxCount++] = rx_data;
		
		if(K230_RxCount >= 6)
		{
			if(rx_data == K230_TAIL)
			{
				Serial_RxFlag = 1;
			}
			K230_RxCount = 0;
		}
	}
	
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);
}
