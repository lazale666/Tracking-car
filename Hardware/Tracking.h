#ifndef _Tracking_H_
#define _Tracking_H_

void Tracking_Init(void);
void get_dat(uint8_t *tracking_dat);
int8_t get_error(void);
uint8_t stop_get(void);
int8_t Tracking_MPU(void);

#endif
