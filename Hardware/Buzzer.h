#ifndef _Buzzer_H_
#define _Buzzer_H_

#define uint8_t unsigned char

void Buzzer_Init(void);
void Buzzer_open(void);
void Buzzer_open_low(void);
void Buzzer_open_key(uint8_t key_command);

#endif
