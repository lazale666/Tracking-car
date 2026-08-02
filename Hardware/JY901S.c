#include "stm32f10x.h"
#include "JY901S.H"
#include "Delay.h"

float JY901S_Yaw = 0.0f;
float JY901S_Pitch = 0.0f;
float JY901S_Roll = 0.0f;

static uint8_t JY901S_RxBuffer[11];
static uint8_t JY901S_RxCount = 0;

static void JY901S_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void JY901S_ZeroZAxis(void)
{
	uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
	uint8_t calibrate_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
	uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
	
	// 1. 解锁
	for(uint8_t i = 0; i < 5; i++)
	{
		JY901S_SendByte(unlock_cmd[i]);
	}
	Delay_ms(200);
	
	// 2. 校准
	for(uint8_t i = 0; i < 5; i++)
	{
		JY901S_SendByte(calibrate_cmd[i]);
	}
	Delay_ms(3000);
	
	// 3. 保存
	for(uint8_t i = 0; i < 5; i++)
	{
		JY901S_SendByte(save_cmd[i]);
	}
}

void JY901S_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  // PB7 = USART1_TX (JY901S_RX)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  // PB6 = USART1_RX (JY901S_TX)
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t data = USART_ReceiveData(USART1);
		
		if(JY901S_RxCount == 0)
		{
			if(data == 0x55)
			{
				JY901S_RxBuffer[JY901S_RxCount++] = data;
			}
		}
		else if(JY901S_RxCount == 1)
		{
			if(data == 0x53)
			{
				JY901S_RxBuffer[JY901S_RxCount++] = data;
			}
			else
			{
				JY901S_RxCount = 0;
			}
		}
		else
		{
			JY901S_RxBuffer[JY901S_RxCount++] = data;
			if(JY901S_RxCount >= 11)
			{
				JY901S_RxCount = 0;
			}
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void JY901S_GetData(void)
{
	if(JY901S_RxBuffer[0] == 0x55 && JY901S_RxBuffer[1] == 0x53)
	{
		int16_t temp;
		
		temp = (int16_t)(JY901S_RxBuffer[3] << 8 | JY901S_RxBuffer[2]);
		JY901S_Pitch = temp / 32768.0f * 180.0f;
		
		temp = (int16_t)(JY901S_RxBuffer[5] << 8 | JY901S_RxBuffer[4]);
		JY901S_Roll = temp / 32768.0f * 180.0f;
		
		temp = (int16_t)(JY901S_RxBuffer[7] << 8 | JY901S_RxBuffer[6]);
		JY901S_Yaw = temp / 32768.0f * 180.0f;
	}
}
