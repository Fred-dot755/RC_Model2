/*********************************************************************************************************************
* 文件名称          pid.h
* 学校名称          沈阳工业大学
* 组织名称          MEIC
* 开发环境          Keil5
* 适用平台          STM32 F407
* 修改记录
* 日期              作者                备注
* 2025-01-11        LZJ.            first version
**********************************************************************************************************************/
#ifndef __PID_H
#define __PID_H
#include "main.h"



typedef struct
{
	float Kp;				//比例
	float Ki;				//积分
	float Kd;				//微分
	float SetSpeed;			//设定值
	float AcutualSpeed;		//实际值
	float err;				//偏差值
	float err_last;			//上一个偏差值
	float I_Max;			//积分上限
	float V_Max;			//输出上限
	float voltage;			//定义电压值（控制执行器的变量）
	float interal;			//定义积分值
	int   pid_speed_result;	//速度PID计算结果
	int   pid_loc_result;   //位置PID计算结果
}pid_struct;

extern pid_struct pid_3508[8];

void PID_Init(pid_struct *pid);

float PID_Calc(pid_struct *pid,float kp,float ki,float kd,float speed,float feedback_speed);

void position_accumulation(int ID);

#endif

