/*----------------------------------------include--------------------------------------------------*/
#include "stm32f10x.h"      // Device header
#include "Motor.H"          //占用STM 2；PA1(左PWM)、PA2(右PWM)、PA4(左in1)、PA5(左in2)、PA6(右in1)、PA7(右in2)
#include "Timer.H"          //占用STM 1   代替Delay用于按键检测、信息获取
#include "OLED.H"           //占用PB8、PB9（软件I2C）
#include "Buzzer.H"         //占用PB12
#include "Key.H"            //占用PB0、PB1(键码值分别为1、2)
#include "MPU6050.H"        //占用PB10(SCL)、PB11(SDA)(硬件I2C)
#include "Tracking.H"       //占用PA9、PA10、PA11、PA12、PB14、PB15
#include "PID.H"            //无占用  用于实现pid算法
/*------------------------------------------data---------------------------------------------------*/
uint8_t open_flag;//reset
uint8_t oled_mod;//oled_mod-0:MPU6050    run_mod-1:Kp、Ki、Kd、Ks(坡度补偿)、Kt(转弯补偿)    run_mod-2:turn right

uint8_t stop_num_key;//定时器计数值    1-1ms  10ms触发一次 用于按键检测
uint8_t stop_num_seg;//定时器计数值    1-1ms  100ms触发一次 用于采集信息
uint8_t stop_num_mot;//定时器计数值    1-1ms  100ms触发一次 用于实时为电机PWM赋值

uint8_t stop_flag=1;//停止标志位 0-非停止状态 1-停止状态
uint8_t buzzer_flag;//蜂鸣器标志位 停止状态下：1-蜂鸣状态（三声后制0）

uint8_t mot_proc_flag;//电机驱动标志位

uint8_t seg_proc_flag;//信息检测驱动标志位

uint8_t key_proc_flag;//按键检测驱动标志位
uint8_t key_old,key_now,key_down,key_up;//下降沿检测用

uint8_t tracking_dat[]={0,0,0,0,0,0};//红外循迹返回值

int16_t AX, AY, AZ, GX, GY, GZ;//陀螺仪读取值  AX可表示重力沿坡向下分量

uint32_t time_num;//利用tim3来测量pid间隔时间
uint32_t time_dat;//利用tim3来测量pid间隔时间(复制用)

float Kp=0.1;    //pid参数
float Ki=0.1;
float Kd=0.1;
float limit=10.0;

float PID_out;//pid算法输出值

uint8_t speed_0=50;//小车基本速度
uint8_t speed_left;//小车左轮速度
uint8_t speed_right;//小车右轮速度

uint8_t i;//用于所有for循环

uint8_t command_flag=0;//调参标志位  置1时改变key1、key2功能
uint8_t command_option=0;//调参选项  key3更改
float command[4]={0.1,0.1,0.1,10.0};
/*------------------------------------------proc---------------------------------------------------*/
void key_proc(void)//按键检测  key—1为OLED参数界面切换 key-2重置停止标志位
{
	if(key_proc_flag) return;//代替Delay实现防抖
	key_proc_flag=1;
	
	key_now=Key_Num();
	key_down=key_now & (key_old ^ key_now);//检测按键下降沿
	key_old=key_now;
	
	if(command_flag==0)//非调参时key1、key2、key3功能
	{
		if(key_down == 1)//key-1 OLED参数界面切换（切换时顺带清屏）
		{
			oled_mod++;
			oled_mod%=4;
			OLED_Clear();
		}
		if(key_down == 2)//key-2 重置停止标志位（顺带蜂鸣器标志位）
		{
			if(stop_flag)stop_flag=0;
			if(buzzer_flag)buzzer_flag=0;
		}
		if(oled_mod == 1)
		{
			if(key_down == 3)//key-3 进入调参界面
			{
				command_flag=1;
			}
		}
	}
	
	if(command_flag==1)//调参时key1、key2、key3功能
	{
		if(key_down == 1)//key-1 增大参数值
		{
			command[command_option-1]+=0.01;
		}
		if(key_down == 2)//key-2 减小参数值
		{
			command[command_option-1]-=0.01;
		}
		if(key_down == 3)//key-3 切换参数选项
		{
			command_option+=1;
		}
		if(command_option>=5)//调完参数返回
		{
			command_flag=0;
			command_option=0;
			Kp=command[0];
			Ki=command[1];
			Kd=command[2];
			limit=command[3];
		}
	}
	
	
	
}

void seg_proc(void)
{
	if(seg_proc_flag) return;
	seg_proc_flag=1;
	//OLED信息处理部分：
	if(oled_mod==0)//MPU6050参数显示
	{
		OLED_ShowString(1,1,"MPU6050");
		
		OLED_ShowSignedNum(2, 1, AX, 5);
		OLED_ShowSignedNum(3, 1, AY, 5);
		OLED_ShowSignedNum(4, 1, AZ, 5);
		OLED_ShowSignedNum(2, 8, GX, 5);
		OLED_ShowSignedNum(3, 8, GY, 5);
		OLED_ShowSignedNum(4, 8, GZ, 5);
	}
	
	if(oled_mod==1)//pid参数显示&调参
	{
			OLED_ShowString(1,2,"Kp:");
			OLED_ShowSignedNum(1, 8, command[0], 2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(1,12,command[0]*1000,3);
			
			OLED_ShowString(2,2,"Ki:");
			OLED_ShowSignedNum(2, 8, command[1], 2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(2,12,command[1]*1000,3);
			
			OLED_ShowString(3,2,"Kd:");
			OLED_ShowSignedNum(3, 8, command[2], 2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(3,12,command[2]*1000,3);
			
			OLED_ShowString(4,2,"pp:");
			OLED_ShowSignedNum(4, 8, command[3], 2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(4,12,command[3]*1000,3);
			
			if(command_option==0)
			{
				OLED_ShowChar(4,1,' ');
			}
			
			if(command_flag == 1)
			{
				OLED_ShowChar(command_option,1,'>');
				
				if(command_option>1)
				{
					OLED_ShowChar(command_option-1,1,' ');
				}
			}
		}
	
	if(oled_mod==2)//红外循迹检测显示
	{
		OLED_ShowString(1,1,"Trackig:");
		for(i=0;i<6;i++)
		{
			if(tracking_dat[i]==1)
			{
				OLED_ShowNum(2,(2*i)+1,i+1,1);
			}
			else
			{
				OLED_ShowNum(2,(2*i)+1,0,1);
			}
		}
		OLED_ShowString(3,1,"stop:");
		OLED_ShowNum(3,6,stop_flag,1);
		OLED_ShowString(3,8,"buzz:");
		OLED_ShowNum(3,14,buzzer_flag,1);
	}
	if(oled_mod==3)//pid输出及速度
	{
		OLED_ShowString(1,1,"PID:");
		OLED_ShowString(1,9,"AX:");
		
		OLED_ShowSignedNum(2,9,AX,5);
		
		if(PID_out>=0)
		OLED_ShowChar(2,1,'+');
		if(PID_out<0)
		OLED_ShowChar(2,1,'-');
		
		if(PID_out>=0)
		OLED_ShowNum(2,2,PID_out,1);
		if(PID_out<0)
		OLED_ShowNum(2,2,-PID_out,1);
		
		OLED_ShowChar(2,3,'.');
		if(PID_out>=0)
		OLED_ShowNum(2,4,PID_out*1000,3);
		if(PID_out<0)
			OLED_ShowNum(2,4,PID_out*(-1000),3);
		
		OLED_ShowString(3,1,"left :");
		OLED_ShowNum(3,7,speed_left,3);
		
		OLED_ShowString(4,1,"right:");
		OLED_ShowNum(4,7,speed_right,3);
	}
	//红外循迹部分：
	get_dat(tracking_dat);
	//陀螺仪信息接收：
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	//PID信息处理部分：
	time_dat=time_num;
	time_num=0;
	PID_out=PID(Kp,Ki,Kd,time_dat,limit);
	//停止标志检测： 
	if(stop_flag==0)
		stop_flag=stop_get();
}

void mot_proc(void)
{
	if(mot_proc_flag) return;
	mot_proc_flag=1;
	
	if(stop_flag == 0)//运动状态
	{
		speed_left=speed_0 * (1.0+PID_out) * (0.5+((-AX)/2000.0));
		//公式：左轮速度=基本速度*（1+pid算法输出值）*（0.5+坡度补偿值）
		speed_right=speed_0 * (1.0-PID_out) * (0.5+((-AX)/2000.0));
		//公式：右轮速度=基本速度*（1-pid算法输出值）*（0.5+坡度补偿值）
		Motor_Set_left(speed_left);//输入左轮速度
		Motor_Set_right(speed_right);//输入右轮速度
	}
	if(stop_flag == 1)//停车状态
	{
		speed_left=0;
		speed_right=0;
		
		Motor_Set_left(speed_left);
		Motor_Set_right(speed_right);
		
		if(buzzer_flag==0)//蜂鸣器启动
		{
			Buzzer_open();//因为此时所有模块停止运行
			Buzzer_open();//无需考虑时间损耗（不使用定时器延时……）
			Buzzer_open();//所以直接三个函数叠加表示蜂鸣三次（内为Delay延时）
			buzzer_flag=1;//停止鸣叫，将标志位置1
		}
	}
}

/*------------------------------------------main---------------------------------------------------*/
int main(void)
{
	Timer_Init();   //定时器初始化
	Motor_Init();   //电机初始化
	OLED_Init();    //OLED初始化
	Buzzer_Init();  //蜂鸣器初始化
	MPU6050_Init(); //陀螺仪初始化
	Key_Init();     //按键初始化
	Tracking_Init();//红外循迹初始化
	while(1)
	{
		key_proc();   //循环按键检测
		seg_proc();   //循环信息检测
		mot_proc();   //循环电机控制
	}
}
/*------------------------------------------stop---------------------------------------------------*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		time_num++;//测量时间，用于检测pid的dt来求积分
		
		if(++stop_num_key>=10){stop_num_key=0;key_proc_flag=0;}//按键检测延时  10ms
		if(++stop_num_seg>=100){stop_num_seg=0;seg_proc_flag=0;}//信息检测延时  100ms
		if(++stop_num_mot>=100){stop_num_mot=0;mot_proc_flag=0;}//电机速度更新延时  100ms
		
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除中断标志位
	}
}
