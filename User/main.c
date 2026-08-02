/*--------------------------------include----------------------------------*/
#include "stm32f10x.h"      // Device header
#include "Motor.H"          //占用STM 2；PA3(左PWM)、PA2(右PWM)、PA4(左in1)、PA5(左in2)、PA6(右in1)、PA7(右in2)
#include "Timer.H"          //占用STM 1   代替Delay用于按键检测、信息获取
#include "OLED.H"           //占用PB8、PB9（软件I2C）
#include "Buzzer.H"         //占用PB12
#include "Key.H"            //占用PA0、PA1、PA4、PA12(键码值分别为1、2、3、4)
#include "PID.H"            //无占用  用于实现pid算法
#include "Serial.H"         //占用PB10、PB11  串口通信  用于接收K230的hex数据包
#include "JY901S.H"         //占用PB6、PB7    串口通讯  用于接收陀螺仪的hex数据包
/*----------------------------------data-----------------------------------*/
uint8_t run_mod;

uint8_t oled_mod;//oled_mod-0:MPU6050    run_mod-1:Kp、Ki、Kd、Kt(转弯补偿)    run_mod-2:turn right

uint8_t stop_num_key;//定时器计数值    1-1ms  10ms触发一次 用于按键检测
uint8_t stop_num_seg;//定时器计数值    1-1ms  100ms触发一次 用于采集信息
uint8_t stop_num_mot;//定时器计数值    1-1ms  100ms触发一次 用于实时为电机PWM赋值

uint8_t stop_flag=1;//停止标志位 0-非停止状态 1-停止状态
uint8_t buzzer_flag;//蜂鸣器标志位 停止状态下：1-蜂鸣状态（三声后制0）

uint8_t mot_proc_flag;//电机驱动标志位

uint8_t seg_proc_flag;//信息检测驱动标志位

uint8_t key_proc_flag;//按键检测驱动标志位
uint8_t key_old,key_now,key_down,key_up;//下降沿检测用

float angle;//目前的角度
float angle_mod;//偏差的角度

uint8_t speed_0=70;//小车基本速度
uint8_t speed_left;//小车左轮速度
uint8_t speed_right;//小车右轮速度

uint8_t i;//用于所有for循环

uint8_t command_flag=0;//调参标志位  置1时改变key1、key2功能
uint8_t command_option=0;//调参选项  key3更改
float command[4]={0.17,0.01,0.05,0.3};
uint8_t speed_00=70;

uint8_t start_mod=0;//小车行驶方式  0-陀螺仪直线行驶  1-0字行驶  2-8字行驶
uint8_t mod_flag;//锁住按键用于调整start_mod

uint8_t path_state=0;//路径状态机
uint8_t lap_count=0;//圈数计数
float angle_init=0.0f;//初始角度参考
uint8_t point_reached=0;//到达点标志

int16_t mod_vision ,dx_vision ,speed_vision;//K230回传数据

float Kp=0.17;    //pid参数
float Ki=0.01;
float Kd=0.05;
float limit=0.3;

float Kp_angle=0.03;    //pid参数（拐弯时候PID）
float Ki_angle=0.001;
float Kd_angle=0.005;
float limit_angle=0.3;

float PID_out;//pid算法输出值

float PID_out_angle;//拐弯时pid算法输出值

uint32_t time_num;//利用tim3来测量pid间隔时间
uint32_t time_dat;//利用tim3来测量pid间隔时间(复制用)
/*----------------------------------proc-----------------------------------*/
void key_proc(void)//按键检测  key—1为OLED参数界面切换 key-2重置停止标志位
{
	if(key_proc_flag) return;//代替Delay实现防抖
	key_proc_flag=1;
	
	key_now=Key_Num();
	key_down=key_now & (key_old ^ key_now);//检测按键下降沿
	key_old=key_now;
	
	if(command_flag==0 && mod_flag==0)//非调参时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 OLED参数界面切换（切换时顺带清屏）
		{
			oled_mod++;
			oled_mod%=5;
			OLED_Clear();
		}
		if(key_down == 2)//key—2 重置停止标志位（顺带蜂鸣器标志位）
		{
			if(stop_flag)stop_flag = 0;else stop_flag = 1;
			if(buzzer_flag)buzzer_flag=0;else buzzer_flag = 1;
		}
		if(key_down == 3)//key—2 重置角度值（顺带蜂鸣器标志位）
		{
			
		}
		if(oled_mod == 1)
		{
			if(key_down == 3)//key—3 进入oled-1的调参界面
			{
				command_flag=1;
			}
		}
		if(oled_mod == 4)
		{
			if(key_down == 3)//key—4 进入oled-4的模式切换界面
			{
				mod_flag=1;
			}
		}
	}
	
	if(command_flag==1)//调参时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 增大参数值
		{
			command[command_option-1]+=0.01;
		}
		if(key_down == 2)//key—2 减小参数值
		{
			command[command_option-1]-=0.01;
		}
		if(key_down == 3)//key—3 切换参数选项
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
	
	if(mod_flag==1)//调模式mod时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 改变mod值
		{
			start_mod += 1;
			start_mod %= 3;
		}
		if(key_down == 2)//key—2 复位/初始化
		{
			mod_flag = 0;
			if(stop_flag)stop_flag=0;
			if(buzzer_flag)buzzer_flag=0;
		}
	}
}

void seg_proc(void)
{
	if(seg_proc_flag) return;
	seg_proc_flag=1;
	//OLED信息处理部分：
	if(oled_mod==0)//JY901S参数显示
	{
		OLED_ShowString(1,1,"JY901S");
		
		OLED_ShowSignedNum(2,1,angle,5);
		
		OLED_ShowSignedNum(2,8,K230_dx,5);
	}
	
	if(oled_mod==1)//pid参数显示&调参
	{
			OLED_ShowString(1,2,"Kp:");
			OLED_ShowSignedNum(1,8,command[0],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(1,12,command[0]*1000,3);
			
			OLED_ShowString(2,2,"Ki:");
			OLED_ShowSignedNum(2,8,command[1],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(2,12,command[1]*1000,3);
			
			OLED_ShowString(3,2,"Kd:");
			OLED_ShowSignedNum(3,8,command[2],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(3,12,command[2]*1000,3);
			
			OLED_ShowString(4,2,"pp:");
			OLED_ShowSignedNum(4,8,command[3],2);
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
		OLED_ShowString(1,1,"PID:");
		OLED_ShowString(1,9,"JY901S:");
		
		OLED_ShowSignedNum(2,9,angle,5);
		
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
	if(oled_mod==3)//pid输出及速度
	{
		OLED_ShowString(1,1,"PID:");
		OLED_ShowString(1,9,"K230:");
		
		OLED_ShowSignedNum(2,9,K230_dx,5);
		
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
	
	if(oled_mod==4)//v0
	{
		OLED_ShowString(1,1,"start mod:");
		OLED_ShowNum(2,1,start_mod+1,1);
		
		OLED_ShowString(4,1,"stop:");
		OLED_ShowNum(4,6,stop_flag,1);
		OLED_ShowString(4,8,"buzz:");
		OLED_ShowNum(4,14,buzzer_flag,1);
	}
	//视觉数据获取：
	if(Serial_GetRxFlag())
	{
		GetData(&mod_vision ,&dx_vision ,&speed_vision);
	}
	//陀螺仪信息接收：
	JY901S_GetData();
	//偏转角度计算：
	angle = JY901S_Yaw;
	
	if(start_mod == 0)//陀螺仪直线路线 A→B
	{
		if(stop_flag == 0 && path_state == 0)
		{
			// 初始化开始
			angle_init = angle;
			path_state = 1;
			point_reached = 0;
		}
		// 使用K230数据进行循迹，mod_vision控制直行或停止
		if(path_state == 1)
		{
			// 到达B点停止（K230返回stop信号）
			if(mod_vision == 2 && point_reached == 0)
			{
				point_reached = 1;
				buzzer_flag = 0;
				stop_flag = 1;
				path_state = 0;
			}
		}
	}
	else if(start_mod == 1)//0字路线 A→B→C→D→A
	{
		if(stop_flag == 0 && path_state == 0)
		{
			angle_init = angle;
			path_state = 1;
			point_reached = 0;
			lap_count = 0;
		}
		switch(path_state)
		{
			case 1: // A→B
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					// 到达B点后，重新启动继续前进
					// 这里可以添加短暂延时后自动启动
					path_state = 2;
				}
				break;
			case 2: // B→C 半圆
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 3;
				}
				break;
			case 3: // C→D
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 4;
				}
				break;
			case 4: // D→A 半圆
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 0; // 完成一圈
				}
				break;
		}
	}
	else//start_mod = 2    8字路线 A→C→B→D→A
	{
		if(stop_flag == 0 && path_state == 0)
		{
			angle_init = angle;
			path_state = 1;
			point_reached = 0;
			lap_count = 0;
		}
		switch(path_state)
		{
			case 1: // A→C
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 2;
				}
				break;
			case 2: // C→B 半圆
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 3;
				}
				break;
			case 3: // B→D
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 4;
				}
				break;
			case 4: // D→A 半圆
				if(mod_vision == 2 && point_reached == 0)
				{
					point_reached = 1;
					buzzer_flag = 0;
					stop_flag = 1;
					lap_count++;
					if(lap_count >= 2) // 完成4圈后停止
					{
						path_state = 0;
					}
					else
					{
						path_state = 1;
					}
				}
				break;
		}
	}
	
	//PID信息处理部分：
	time_dat=time_num;
	time_num=0;
	PID_out=PID(Kp,Ki,Kd,time_dat,limit);
	PID_out_angle=PID_angle(Kp_angle,Ki_angle,Kd_angle,time_dat,limit_angle,angle);
	// 重置point_reached以便下一个点检测
	if(mod_vision != 2)
	{
		point_reached = 0;
	}
	//转弯&直行检测： 
	run_mod = mod_vision;
}

void mot_proc(void)
{
	if(mot_proc_flag) return;
	mot_proc_flag=1;
	
	if(stop_flag == 0)//运动状态
	{
		if(run_mod == 0)	// 直线
		{
			speed_left=speed_0 * (0.5+PID_out);
			//公式：左轮速度=基本速度*（1+pid算法输出值）
			speed_right=speed_0 * (0.5-PID_out);
			//公式：右轮速度=基本速度*（1-pid算法输出值
			Motor_Set_left(speed_left);//输入左轮速度
			Motor_Set_right(speed_right);//输入右轮速度
		}
		if(run_mod == 1)	// 循迹
		{
			speed_left=speed_0 * (0.7+PID_out_angle);
			//公式：左轮速度=基本速度*（1+pid算法输出值）
			speed_right=speed_0 * (0.7-PID_out_angle);
			//公式：右轮速度=基本速度*（1-pid算法输出值
			Motor_Set_left(speed_left);//输入左轮速度
			Motor_Set_right(speed_right);//输入右轮速度
		}
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
			Buzzer_open();//所以直接三个函数叠加表示蜂鸣三声（内为Delay延时）
			buzzer_flag=1;//停止鸣叫，将标志位置1
		}
	}
}

/*----------------------------------main-----------------------------------*/
int main(void)
{
	Timer_Init();   //定时器初始化
	Motor_Init();   //电机初始化
	OLED_Init();    //OLED初始化
	Buzzer_Init();  //蜂鸣器初始化
	Key_Init();     //按键初始化
	JY901S_Init(); //陀螺仪初始化
	Serial_Init();  //串口初始化
	
	while(1)
	{
		key_proc();   //循环按键检测
		seg_proc();   //循环信息检测
		mot_proc();   //循环电机控制
	}
}
/*----------------------------------stop-----------------------------------*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		time_num++;//测量时间，用于检测pid的dt来求积分
		
		if(++stop_num_key>=10){stop_num_key=0;key_proc_flag=0;}//按键检测延时  10ms
		{stop_num_seg=0;seg_proc_flag=0;}//信息检测延时  100ms
		{stop_num_mot=0;mot_proc_flag=0;}//电机速度更新延时  100ms
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除中断标志位
	}
}
