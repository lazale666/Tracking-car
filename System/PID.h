#ifndef _PID_H_
#define _PID_H_

float PID(float Kp,float Ki,float Kd,uint16_t time_dat,float limit);
float PID_angle(float Kp,float Ki,float Kd,uint16_t time_dat,float limit,float angle);

#endif
