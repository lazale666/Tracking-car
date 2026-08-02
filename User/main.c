/*--------------------------------include----------------------------------*/
#include "stm32f10x.h"      // Device header
#include "Motor.H"          //占用PA8、PB13、PB14、PB15(PA8左in1、PB15左in2、PB14右in1、PB13右in2)
#include "Timer.H"          //占用STM 1   代替Delay用于按键检测、信息获取
#include "OLED.H"           //占用PB8、PB9（软件I2C）
#include "Buzzer.H"         //占用PB12
#include "Key.H"            //占用PA0、PA1、PA4、PA12(键码值分别为1、2、3、4)
#include "PID.H"            //无占用  用于实现pid算法
#include "Serial.H"         //占用PB10、PB11  串口通信  用于接收K230的hex数据包
#include "JY901S.H"         //占用PB6、PB7    串口通讯  用于接收陀螺仪的hex数据包

#define RUN_MODE_STOP      0
#define RUN_MODE_TRACK     1
#define RUN_MODE_STRAIGHT  3
#define VISION_MODE_STOP   5
#define VISION_RELEASE_COUNT  3

#define TRACK_BASE_RATE       0.4f
#define STRAIGHT_BASE_RATE    0.7f
#define STRAIGHT_TURN_GAIN  120.0f
#define MIN_TURN_PWM          2.0f
/*----------------------------------data-----------------------------------*/
uint8_t run_mod;

uint8_t oled_mod;//oled_mod-0:MPU6050    run_mod-1:Kp、Ki、Kd、Kt(转弯补偿)    run_mod-2:turn right

uint8_t stop_num_key;//定时器计数值    1-1ms  10ms触发一次 用于按键检测
uint8_t stop_num_seg;//定时器计数值    1-1ms  20ms触发一次 用于采集信息
uint8_t stop_num_mot;//定时器计数值    1-1ms  20ms触发一次 用于实时为电机PWM赋值
uint8_t stop_num_buzzer;//定时器计数值    1-1ms  100ms触发一次 用于触发蜂鸣器

uint8_t stop_flag=1;//停止标志位 0-非停止状态 1-停止状态
uint8_t buzzer_flag=1;//蜂鸣器标志位 停止状态下：1-蜂鸣状态（三声后制0）

uint8_t buzzer_proc_flag;//蜂鸣器驱动标志位

uint8_t mot_proc_flag;//电机驱动标志位

uint8_t seg_proc_flag;//信息检测驱动标志位

uint8_t key_proc_flag;//按键检测驱动标志位
uint8_t key_old,key_now,key_down;//下降沿检测用

float angle;//目前的角度
float angle_mod;//偏差的角度

float speed_0=30.0;//小车基本速度
int8_t speed_left;//小车左轮速度
int8_t speed_right;//小车右轮速度
float speed_l;
float speed_r;

uint8_t command_flag=0;//调参标志位  置1时改变key1、key2功能
uint8_t command_option=0;//调参选项  key3更改
float command[4]={0.07,0.01,0.11,0.3};
float command_angle[4]={0.12,0.001,0.005,0.3};

uint8_t start_mod=0;//小车行驶方式  0-陀螺仪直线行驶  1-0字行驶  2-8字行驶
uint8_t mod_flag;//锁住按键用于调整start_mod

uint8_t path_state=0;//路径状态机
uint8_t lap_count=0;//圈数计数
float angle_init=0.0f;//初始角度参考
uint8_t point_reached=0;//到达点标志
uint8_t vision_release_count=0;//到点信号离开确认计数
uint8_t last_path_state=0;//用于检测路径状态切换

int16_t mod_vision ,dx_vision ,speed_vision;//K230回传数据

float Kp=0.12;    //pid参数
float Ki=0.01;
float Kd=0.03;
float limit=0.3;

float Kp_angle=0.05;    //pid参数（拐弯时候PID）
float Ki_angle=0.001;
float Kd_angle=0.11;
float limit_angle=0.3;

float PID_out;//pid算法输出值

float PID_out_angle;//拐弯时pid算法输出值

uint32_t time_num;//利用tim3来测量pid间隔时间
uint32_t time_dat;//利用tim3来测量pid间隔时间(复制用)
uint8_t display_divider;//降低OLED刷新频率，避免拖慢控制环

uint8_t count = 0;

static int8_t speed_limit(float speed)
{
	if(speed > 100.0f)return 100;
	if(speed < -100.0f)return -100;
	return (int8_t)speed;
}

static float turn_pwm_limit(float turn)
{
	if(turn > 100.0f)return 100.0f;
	if(turn < -100.0f)return -100.0f;
	return turn;
}

static void apply_min_turn(float *turn)
{
	if(*turn > 0.0f && *turn < MIN_TURN_PWM)*turn = MIN_TURN_PWM;
	if(*turn < 0.0f && *turn > -MIN_TURN_PWM)*turn = -MIN_TURN_PWM;
}

static void mark_point_reached(void)
{
	point_reached = 1;
	vision_release_count = 0;
}

static void reset_point_detector(void)
{
	point_reached = 0;
	vision_release_count = 0;
}
/*----------------------------------proc-----------------------------------*/
void key_proc(void)//按键检测  key—1为OLED参数界面切换 key-2重置停止标志位
{
	if(key_proc_flag) return;//代替Delay实现防抖
	key_proc_flag=1;
	
	key_now=Key_Num();
	key_down=key_now & (key_old ^ key_now);//检测按键下降沿
	key_old=key_now;

	if(key_down == 2 && stop_flag == 0)
	{
		stop_flag = 1;
		return;
	}
	
	if(command_flag==0 && mod_flag==0)//非调参时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 OLED参数界面切换（切换时顺带清屏）
		{
			oled_mod++;
			oled_mod%=7;
			OLED_Clear();
		}
		if(key_down == 2)//key—2 启停控制（顺带蜂鸣器标志位）
		{
			if(stop_flag)
			{
				stop_flag = 0;
				buzzer_flag = 0;
			}
			else
			{
				stop_flag = 1;
			}
		}
		
		if(oled_mod == 1)
		{
			if(key_down == 3)//key—3 进入oled-1的调参界面
			{
				command_flag=1;
				command_option=0;
			}
		}
		if(oled_mod == 2)
		{
			if(key_down == 3)//key—3 进入oled-1的调参界面
			{
				command_flag=2;
				command_option=0;
			}
		}
		if(oled_mod == 5)
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
	
	if(command_flag==2)//调PID_angle参时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 增大参数值
		{
			command_angle[command_option-1]+=0.01;
		}
		if(key_down == 2)//key—2 减小参数值
		{
			command_angle[command_option-1]-=0.01;
		}
		if(key_down == 3)//key—3 切换参数选项
		{
			command_option+=1;
		}
		if(command_option>=5)//调完参数返回
		{
			command_flag=0;
			command_option=0;
			Kp_angle=command_angle[0];
			Ki_angle=command_angle[1];
			Kd_angle=command_angle[2];
			limit_angle=command_angle[3];
		}
	}
	
	if(mod_flag==1)//调模式mod时key1、key2、key3功能
	{
		if(key_down == 1)//key—1 改变mod值
		{
			start_mod += 1;
			start_mod %= 4;
		}
		if(key_down == 2)//key—2 复位/初始化
		{
			mod_flag = 0;
			path_state = 0;
			last_path_state = 0;
			reset_point_detector();
			if(stop_flag)stop_flag=0;
			if(buzzer_flag)buzzer_flag=0;
		}
	}
}

void seg_proc(void)
{
	if(seg_proc_flag) return;
	seg_proc_flag=1;
	display_divider++;
	if(display_divider >= 5)
	{
	display_divider = 0;
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
	
	if(oled_mod==2)//pid_angle参数显示&调参
	{
			OLED_ShowString(1,2,"Kp:");
			OLED_ShowSignedNum(1,8,command_angle[0],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(1,12,command_angle[0]*1000,3);
			
			OLED_ShowString(2,2,"Ki:");
			OLED_ShowSignedNum(2,8,command_angle[1],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(2,12,command_angle[1]*1000,3);
			
			OLED_ShowString(3,2,"Kd:");
			OLED_ShowSignedNum(3,8,command_angle[2],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(3,12,command_angle[2]*1000,3);
			
			OLED_ShowString(4,2,"pp:");
			OLED_ShowSignedNum(4,8,command_angle[3],2);
			OLED_ShowChar(1,11,'.');
			OLED_ShowNum(4,12,command_angle[3]*1000,3);
			
			if(command_option==0)
			{
				OLED_ShowChar(4,1,' ');
			}
			
			if(command_flag == 2)
			{
				OLED_ShowChar(command_option,1,'>');
				
				if(command_option>1)
				{
					OLED_ShowChar(command_option-1,1,' ');
				}
			}
	}
	
	if(oled_mod==3)//陀螺仪检测显示
	{
		OLED_ShowString(1,1,"PID:");
		OLED_ShowString(1,9,"JY901S:");
		
		OLED_ShowSignedNum(2,9,angle_mod,5);
		
		if(PID_out_angle>=0)
		OLED_ShowChar(2,1,'+');
		if(PID_out_angle<0)
		OLED_ShowChar(2,1,'-');
		
		if(PID_out_angle>=0)
		OLED_ShowNum(2,2,PID_out_angle,1);
		if(PID_out_angle<0)
		OLED_ShowNum(2,2,-PID_out_angle,1);
		
		OLED_ShowChar(2,3,'.');
		if(PID_out_angle>=0)
		OLED_ShowNum(2,4,PID_out_angle*1000,3);
		if(PID_out_angle<0)
			OLED_ShowNum(2,4,PID_out_angle*(-1000),3);
		
		OLED_ShowString(3,1,"left :");
		OLED_ShowNum(3,7,speed_left,3);
		
		OLED_ShowString(4,1,"right:");
		OLED_ShowNum(4,7,speed_right,3);
	}
	if(oled_mod==4)//pid输出及速度
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
	
	if(oled_mod==5)//start mod
	{
		OLED_ShowString(1,1,"start mod:");
		OLED_ShowNum(2,1,start_mod+1,1);
		
		OLED_ShowString(4,1,"stop:");
		OLED_ShowNum(4,6,stop_flag,1);
		OLED_ShowString(4,8,"buzz:");
		OLED_ShowNum(4,14,buzzer_flag,1);
	}
	
	if(oled_mod==6)//k230 max
	{
		OLED_ShowString(1,1,"vision mod:");
		OLED_ShowNum(2,1,mod_vision,2);
		
		OLED_ShowString(3,1,"dx:");
		OLED_ShowNum(4,1,dx_vision,5);
	}
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

	run_mod = RUN_MODE_STOP;

	if(start_mod == 0)//陀螺仪直线路线 A→B
	{
		if(stop_flag == 0 && path_state == 0)
		{
			// 初始化开始
			angle_init = angle;
			path_state = 1;
			last_path_state = 1;
			reset_point_detector();
		}
		if(path_state == 1)
		{
			run_mod = RUN_MODE_STRAIGHT;
			angle_mod = angle_do(angle - angle_init);
			// 到达B点停止（K230返回stop信号）
			if(mod_vision == VISION_MODE_STOP && point_reached == 0)
			{
				mark_point_reached();
				buzzer_flag = 0;
				stop_flag = 1;
				path_state = 0;
				last_path_state = 0;
			}
		}
	}
	else if(start_mod == 1)//0字路线 A→B→C→D→A
	{
		if(stop_flag == 0 && path_state == 0)
		{
			angle_init = angle;
			path_state = 1;
			last_path_state = 1;
			reset_point_detector();
			lap_count = 0;
		}
		if(path_state != last_path_state)
		{
			if(path_state == 1) // || path_state == 3)
			{
				angle_init = angle;
			}
			if(path_state == 3)
			{
				angle_init = angle_init + 180.0f; // C点相对于A点180度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			last_path_state = path_state;
		}
		switch(path_state)
		{
			case 1: // A→B
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					path_state = 2;
					buzzer_flag = 0;
				}
				break;
			case 2: // B→C 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					path_state = 3;
					buzzer_flag = 0;
				}
				break;
			case 3: // C→D
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					path_state = 4;
					buzzer_flag = 0;
				}
				break;
			case 4: // D→A 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 0; // 完成一圈
					last_path_state = 0;
				}
				break;
		}
	}
	else if(start_mod == 2)//start_mod = 2    8字路线 A→C→B→D→A
	{
		if(stop_flag == 0 && path_state == 0)
		{
			angle_init = angle;
			path_state = 1;
			last_path_state = 1;
			reset_point_detector();
			lap_count = 0;
		}
		if(path_state != last_path_state)
		{
			if(path_state == 1)// || path_state == 3)
			{
				angle_init = angle;
			}
			if(path_state == 2)
			{
				angle_init = angle_init + 38.66f; // B点相对于A点顺时针38.66度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			if(path_state == 4)
			{
				angle_init = angle_init - 102.68f - 38.66f; // C点相对于A点逆时针90度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			if(path_state == 5)
			{
				angle_init = angle_init - 38.66f; // C点相对于A点逆时针90度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			last_path_state = path_state;
		}
		switch(path_state)
		{
			case 1: // A→C
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					path_state = 2;
					buzzer_flag = 0;
				}
				break;
			case 2: // 转弯缓冲
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(angle_mod < 2.0f && angle_mod > -2.0f)//if条件应为小车正朝入弯切线方向
				{
					mark_point_reached();
					path_state = 3;
				}
				break;
			case 3: // C→B 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					path_state = 4;
					buzzer_flag = 0;
				}
				break;
			case 4: // B→D
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)//if条件应为小车正朝入弯切线方向
				{
					path_state = 5;
					buzzer_flag = 0;
				}
				break;
			case 5: // 转弯缓冲
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(angle_mod < 2.0f && angle_mod > -2.0f)//if条件应为小车正朝入弯切线方向
				{
					mark_point_reached();
					path_state = 6;
				}
				break;
			case 6: // D→A 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					buzzer_flag = 0;
					stop_flag = 1;
					path_state = 0;
					last_path_state = 0;
				}
				break;
		}
	}
	else//start_mod = 3  循环8字
	{
		if(stop_flag == 0 && path_state == 0)
		{
			angle_init = angle;
			path_state = 1;
			last_path_state = 1;
			reset_point_detector();
			lap_count = 0;
		}
		if(path_state != last_path_state)
		{
			if(path_state == 1 && lap_count == 0)
			{
				angle_init = angle;
			}
			if(path_state == 1 && lap_count >= 1)
			{
				angle_init = angle_init + 102.68f + 38.66f + 4.0f; // AC路线相对于入弯切线顺时针102.68+38.66度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			if(path_state == 2)
			{
				angle_init = angle_init + 38.66f - 4.0f; // 入弯切线相对于AC路线顺时针38.66度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			if(path_state == 4)
			{
				angle_init = angle_init - 102.68f - 38.66f - 4.0f; // BD路线相对于入弯切线逆时针102.68+38.66度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			if(path_state == 5)
			{
				angle_init = angle_init - 38.66f + 4.0f; // 入弯切线相对于BD路线顺时针38.66度
				if(angle_init > 180.0f)angle_init -= 360.0f;
				if(angle_init < -180.0f)angle_init += 360.0f;
			}
			last_path_state = path_state;
		}
		switch(path_state)
		{
			case 1: // A→C
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					path_state = 2;
				}
				break;
			case 2: // 转弯缓冲
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(angle_mod < 2.0f && angle_mod > -2.0f)//if条件应为小车正朝入弯切线方向
				{
					mark_point_reached();
					path_state = 3;
				}
				break;
			case 3: // C→B 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();
					path_state = 4;
				}
				break;
			case 4: // B→D
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)//if条件应为小车正朝入弯切线方向
				{
					path_state = 5;
				}
				break;
			case 5: // 转弯缓冲
				run_mod = RUN_MODE_STRAIGHT;
				angle_mod = angle_do(angle - angle_init);
				if(angle_mod < 2.0f && angle_mod > -2.0f)//if条件应为小车正朝入弯切线方向
				{
					mark_point_reached();
					path_state = 6;
				}
				break;
			case 6: // D→A 半圆
				run_mod = RUN_MODE_TRACK;
				if(mod_vision == VISION_MODE_STOP && point_reached == 0)
				{
					mark_point_reached();

					lap_count++;
					if(lap_count >= 4) // 完成4圈后停止
					{
						path_state = 0;
						last_path_state = 0;
						buzzer_flag = 0;
						stop_flag = 1;
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
	PID_out_angle=PID_angle(Kp_angle,Ki_angle,Kd_angle,time_dat,limit_angle,-angle_mod);
	// 重置point_reached以便下一个点检测
	if(mod_vision == VISION_MODE_STOP)
	{
		vision_release_count = 0;
	}
	else if(point_reached)
	{
		vision_release_count++;
		if(vision_release_count >= VISION_RELEASE_COUNT)
		{
			point_reached = 0;
			vision_release_count = 0;
		}
	}
}

void mot_proc(void)
{
	float base;
	float turn;

	if(mot_proc_flag) return;
	mot_proc_flag=1;
	
	if(stop_flag == 0)//运动状态
	{
		if(run_mod == RUN_MODE_TRACK)	// 循迹
		{
			base = speed_0 * TRACK_BASE_RATE;
			turn = speed_0 * PID_out;
			apply_min_turn(&turn);
			turn = turn_pwm_limit(turn);

			speed_l = base + turn;
			speed_left = speed_limit(speed_l);
			speed_r = base - turn;
			speed_right = speed_limit(speed_r);
			Motor_Set_left(speed_left);//输入左轮速度
			Motor_Set_right(speed_right);//输入右轮速度
		}
		else if(run_mod == RUN_MODE_STRAIGHT)	// 直线
		{
			base = speed_0 * STRAIGHT_BASE_RATE;
			turn = STRAIGHT_TURN_GAIN * PID_out_angle;
			apply_min_turn(&turn);
			turn = turn_pwm_limit(turn);

			speed_l = base + turn;
			speed_left = speed_limit(speed_l);
			speed_r = base - turn;
			speed_right = speed_limit(speed_r);
			Motor_Set_left(speed_left);//输入左轮速度
			Motor_Set_right(speed_right);//输入右轮速度
		}
		else
		{
			speed_left=0;
			speed_right=0;
			Motor_Set_left(speed_left);
			Motor_Set_right(speed_right);
		}
	}
	if(stop_flag == 1)//停车状态
	{
		speed_left=0;
		speed_right=0;
		
		Motor_Set_left(speed_left);
		Motor_Set_right(speed_right);
	}
}

void buzzer_proc()
{
	if(buzzer_proc_flag) return;
	buzzer_proc_flag=1;
	
	buzzer_set(count % 2);
	
	count++;
	
	if(count >= 7){count = 0;buzzer_flag = 1;}
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
		buzzer_proc();
	}
}
/*----------------------------------stop-----------------------------------*/
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		time_num++;//测量时间，用于检测pid的dt来求积分
		
		if(++stop_num_key>=10){stop_num_key=0;key_proc_flag=0;}//按键检测延时  10ms
		if(++stop_num_seg>=20){stop_num_seg=0;seg_proc_flag=0;}//信息检测延时  20ms
		if(++stop_num_mot>=20){stop_num_mot=0;mot_proc_flag=0;}//电机速度更新延时  20ms
		
		if(buzzer_flag==0){if(++stop_num_buzzer>=100){stop_num_buzzer=0;buzzer_proc_flag=0;}}
		
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除中断标志位
	}
}
