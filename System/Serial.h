#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern uint8_t Serial_TxPacket[];
extern uint8_t Serial_RxPacket[];
extern int16_t K230_Mod;
extern int16_t K230_dx;
extern int16_t K230_Speed;

void Serial_Init(void);
uint8_t Serial_GetRxFlag(void);
void GetData(int16_t *Mode, int16_t *dx, int16_t *Speed);
void Serial_SendByte(uint8_t Byte);
void Serial_SendPacket(void);

#endif
