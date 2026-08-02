#include "stm32f10x.h"                  // Device header
#include "Serial.H"

extern int16_t K230_dx;

float PID(float Kp,float Ki,float Kd,uint16_t time_dat,float limit)//pid算法
{
	float p,i,d,out;
	static float integral = 0.0f;//积分
	float derivative;//微分
	static float error_old;//上一次误差值
	
	static float filtered_deriv;//滤波
	
	float error = K230_dx * 0.03;
	//求比例项：
	p=Kp * error;
	//求积分项：
	integral+=(time_dat * 0.001) * error;
	if(integral>limit)integral=limit;  //限幅
	if(integral<-limit)integral=-limit;//限幅
	i=Ki * integral;
	//求微分项：(带防突变滤波)
	if(time_dat>1)//避免除0
	{
		derivative=(error-error_old)/time_dat;
		
		filtered_deriv=0.3 * derivative + 0.7 * filtered_deriv;
		derivative=filtered_deriv;
	}
	
	error_old=error;
	d=Kd * derivative;
	//计算输出值：
	out=p + i + d;
	//输出限幅：
	if(out>(limit * 2))out=(limit * 2);
	if(out<-(limit * 2))out=-(limit * 2);
	
	return out;
}

float PID_angle(float Kp,float Ki,float Kd,uint16_t time_dat,float limit,float angle)//pid算法
{
	float p,i,d,out;
	static float integral = 0.0f;//积分
	float derivative;//微分
	static float error_old;//上一次误差值
	
	static float filtered_deriv;//滤波
	
	float error = angle * 0.1;
	//求比例项：
	p=Kp * error;
	//求积分项：
	integral+=(time_dat * 0.001) * error;
	if(integral>limit)integral=limit;  //限幅
	if(integral<-limit)integral=-limit;//限幅
	i=Ki * integral;
	//求微分项：(带防突变滤波)
	if(time_dat>1)//避免除0
	{
		derivative=(error-error_old)/time_dat;
		
		filtered_deriv=0.3 * derivative + 0.7 * filtered_deriv;
		derivative=filtered_deriv;
	}
	
	error_old=error;
	d=Kd * derivative;
	//计算输出值：
	out=p + i + d;
	//输出限幅：
	if(out>(limit * 2))out=(limit * 2);
	if(out<-(limit * 2))out=-(limit * 2);
	
	return out;
}

float angle_do(float angle)
{
	if(angle >  180.0){angle-=360.0;}
	if(angle < -180.0){angle+=360.0;}
	return angle;
}
