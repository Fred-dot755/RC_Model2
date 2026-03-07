#include "main.h"

pid_struct pid_3508[8] = {0};

void PID_Init(pid_struct *pid)
{
    pid->Kp				=	0;
    pid->Ki				=	0;
    pid->Kd				=	0;

	pid-> Kp            =	0;			
	pid-> Ki			=	0;		
	pid-> Kd	 		=	0;
	pid-> SetSpeed 		=	0;
	pid-> AcutualSpeed  =	0;	
	pid-> err			=	0;
	pid-> err_last		=	0;
	pid-> I_Max			=	10000;
	pid-> V_Max			=	10000;
	pid-> voltage		=	0;	
	pid-> interal		=	0;
	pid->   pid_speed_result=0;	
	pid->   pid_loc_result=0;
}


float PID_Calc(pid_struct *pid,float kp,float ki,float kd,float speed,float feedback_speed)
{
    pid->Kp				=	kp;
    pid->Ki				=	ki;
    pid->Kd				=	kd;
    pid->AcutualSpeed	=	feedback_speed;   						//反馈实际速度
    pid->SetSpeed		=	speed;									//设定速度
    pid->err			=	pid->SetSpeed	-	pid->AcutualSpeed;	//差值

    if(pid->interal > pid->I_Max)									//定义积分值大于积分上限
	{
		pid->interal	=	pid->I_Max;								//积分上限值赋予定义积分值
	}
    else if(pid->interal < -pid->I_Max)								//定义积分值小于负的积分上限
	{
		pid->interal	=	-pid->I_Max;							//负的积分上限值赋予定义积分值
	}

    pid->interal	+=	pid->err;									//定义积分值累加差值
	
    pid->voltage	=		pid->Kp * pid->err + pid->Ki * pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）

    pid->err_last	=		pid->err;								//上一个偏差值

    if(pid->voltage > pid->V_Max)
	{
		pid->voltage	=	pid->V_Max;								//定义电压值大于输出上限
	}
    else if(pid->voltage < -pid->V_Max)
	{
		pid->voltage	=	-pid->V_Max;
	}

    return pid->voltage;                                          //返回定义电压值
}

void position_accumulation(int ID)
{


		if(motor[ID].NowSpeed>0)
		{
			if(motor[ID].NowEnconder>motor[ID].LastEnconder)
			{
				motor[ID].accumulated_distance+=(motor[ID].NowEnconder-motor[ID].LastEnconder);
			}
			else
			{
				if((motor[ID].NowEnconder-motor[ID].LastEnconder)>4000||(motor[ID].NowEnconder-motor[ID].LastEnconder)<-4000)
				{
					motor[ID].accumulated_distance+=(motor[ID].NowEnconder+(8191-motor[ID].LastEnconder));
					
				}
				else
				{
					motor[ID].accumulated_distance=motor[ID].accumulated_distance;
				}
			}
		}
		else if(motor[ID].NowSpeed<-0)
		{
			if(motor[ID].NowEnconder>motor[ID].LastEnconder)
			{
				if((motor[ID].NowEnconder-motor[ID].LastEnconder)>4000||(motor[ID].NowEnconder-motor[ID].LastEnconder)<-4000)
				{
					motor[ID].accumulated_distance-=(motor[ID].LastEnconder+(8191-motor[ID].NowEnconder));
				}
				else
				{
					motor[ID].accumulated_distance=motor[ID].accumulated_distance;
				}
			}
			else
			{
				motor[ID].accumulated_distance-=(motor[ID].LastEnconder-motor[ID].NowEnconder);
			}
		}
	
}

