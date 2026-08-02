#ifndef _JY901S_H_
#define _JY901S_H_

#include "stm32f10x.h"

extern float JY901S_Yaw;
extern float JY901S_Pitch;
extern float JY901S_Roll;

void JY901S_Init(void);
void JY901S_GetData(void);
void JY901S_ZeroZAxis(void);

#endif
