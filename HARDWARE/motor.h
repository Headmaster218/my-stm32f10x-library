/*
 * @Author: error: git config user.name && git config user.email & please set dead value or install git
 * @Date: 2022-11-02 21:09:25
 * @LastEditors: error: git config user.name && git config user.email & please set dead value or install git
 * @LastEditTime: 2022-11-08 18:42:52
 * @FilePath: \USERd:\STM32\My Project\Balance_Car\HARDWARE\motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef __MOTOR_H
#define __MOTOR_H
#include "my_math.h"

#define CYCLE_MS 10							   //控制周期
#define SOFT_START_DIS 100					   //缓启停距离
#define ANGLE_TO_CAP 3.1					   //角度转换为脉冲 
#define MM_TO_CAP (13 * 30) / (WHEEL_D * 3.14) //毫米转换为脉冲
#define WHEEL_D 75							   //轮子直径
// motor 1:30
// 编码器 13/R
// (pi*75)/390 = 0.6041524230769231mm/encoder

//电机数据结构体
struct _motor
{
	char num; //电机编号
	double ave_spd;
	double now_spd;			 //瞬时速度
	double target_spd;		 //目标速度
	short last_spd;			 //计算变量
	unsigned int last_time;	 //编码器计算变量
	unsigned int now_time;	 //编码器触发时间
	short output_spd;		 //输出PWM速度
	double acc;				 //加速度
	long distance;			 //总编码器脉冲
	long start_distance;	 //单次行走开始时编码器脉冲
	long distance_target;	 //目标编码器脉冲
	long target_movement;	 //目标行走距离
	long distance_mm;		 //总距离
	long distance_mm_target; //目标距离
	double complete_percent; //完成度
	short dir;				 //方向
	u8 cap_time;			 // 13一圈
};

union _u16_bit
{
	struct
	{
		u16 ch0 : 1;
		u16 ch1 : 1;
		u16 ch2 : 1;
		u16 ch3 : 1;
		u16 ch4 : 1;
		u16 ch5 : 1;
		u16 ch6 : 1;
		u16 ch7 : 1;
		u16 ch8 : 1;
		u16 ch9 : 1;
		u16 ch10 : 1;
		u16 ch11 : 1;
		u16 ch12 : 1;
		u16 ch13 : 1;
		u16 ch14 : 1;
		u16 ch15 : 1;
	} bit;
	u16 data;
};

union _u8_bit
{
	struct
	{
		u8 ch0 : 1;
		u8 ch1 : 1;
		u8 ch2 : 1;
		u8 ch3 : 1;
		u8 ch4 : 1;
		u8 ch5 : 1;
		u8 ch6 : 1;
		u8 ch7 : 1;
	} bit;
	u8 data;
};


void Car_Rotate(float angle);
void Car_Move_Dir(short distance, u8 dir);
void Wheel_Move(struct _motor *motorx);
void Car_Set_Move(short A, short B, short C, short D);
void Car_Move(void);
void Adder_PID(void);
void Motor_Set_Output(short spd, char dir);
void Motor_Direction_Init(void);
void Motor_PWM_Init(u16 arr, u16 psc);
void Motor_Cap_Init(u16 arr, u16 psc);
void Motor_Init(void);

#endif
