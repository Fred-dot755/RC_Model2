/*
 * QXL_Chassis.c
 *
 *  Created on: May 3, 2026
 *      Author: zsh
 */
#include "QXL_Chassis.h"

QXL_chassis_t QXL_Chassis_Controller;
/*
 * 在此处设置你的底盘参数 a b   ，P，Q,R矩阵
 * a,b介绍在头文件中
 *
 */
const float chassis_a=0.155*sqrt_2f/2.f;
const float chassis_b=0.155*sqrt_2f/2.f;
const float g=9.82;

float Q_data[3*3]={
	10,   0,   0,
	0,   10,   0,
	0,   0,   10
};

float R_data[5*5]={
	100, 0,   0,   0,   0,
	0,   100, 0,   0,   0,
	0,   0,   100, 0,   0,
	0,   0,   0,   100, 0,
	0,   0,   0,   0,   4
};
float P_data[3*3]={
	100000, 0,     0,
	0     , 100000, 0,
	0     ,0    , 10000
};
/*----------------------------------------------------------------------------*/

//实例化底盘对象
QXL_chassis Qxl_chassis(chassis_a,chassis_b,Q_data,R_data,P_data);


void QXL_Chassis_Init(QXL_chassis_t* QXL_Chassis)
{
	QXL_Chassis->QXL_Chassis_Observe.a=chassis_a;
	QXL_Chassis->QXL_Chassis_Observe.b=chassis_b;

	QXL_Chassis->QXL_Chassis_Observe.r=sqrtf(chassis_a*chassis_a+chassis_b*chassis_b);

//	QXL_Chassis->QXL_Chassis_Observe.N=60;

}

void QXL_chassis_observe_update(float v[4],float gyro_yaw, QXL_chassis_t* QXL_Chassis,float dt)
{
	for(int i=0;i<4;i++)
		QXL_Chassis->QXL_Chassis_Observe.v[i]=v[i];
	QXL_Chassis->QXL_Chassis_Observe.gyro_yaw=gyro_yaw;

	Qxl_chassis.Observe_kalman_update(&QXL_Chassis->QXL_Chassis_Observe,dt);
}


void QXL_chassis_controller_init(float vx_kp,float vy_kp,float omega_kp,float omega_ki,float mju,float m,float J,QXL_chassis_t* QXL_Chassis)
{
	QXL_Chassis->QXL_Chassis_Controller.chassis_m= m ;
	QXL_Chassis->QXL_Chassis_Controller.chassis_J= J ;
	QXL_Chassis->QXL_Chassis_Controller.  mju    =mju;

	zxPID_init( vx_kp  , 0       , 0  , 0  , 100, &QXL_Chassis->QXL_Chassis_Controller.QXL_vx_pid   );
	zxPID_init( vy_kp  , 0       , 0  , 0  , 100, &QXL_Chassis->QXL_Chassis_Controller.QXL_vy_pid   );
	zxPID_init(omega_kp, omega_ki, 0  , 100, 100, &QXL_Chassis->QXL_Chassis_Controller.QXL_omega_pid);
}

void QXL_chassis_controller_update(float _target_vx,float _target_vy,float _target_omega,QXL_chassis_t* QXL_Chassis)
{
	QXL_Chassis->QXL_Chassis_Controller.QXL_target_Vx    = _target_vx;
	QXL_Chassis->QXL_Chassis_Controller.QXL_target_Vy    = _target_vy;
	QXL_Chassis->QXL_Chassis_Controller.QXL_target_Omega = _target_omega;

	//计算目标加速度，角加速度
	float target_accel_x  =
			zx_Calc(QXL_Chassis->QXL_Chassis_Controller.QXL_target_Vx, QXL_Chassis->QXL_Chassis_Observe.vx, &QXL_Chassis->QXL_Chassis_Controller.QXL_vx_pid);

	float target_accel_y  =
			zx_Calc(QXL_Chassis->QXL_Chassis_Controller.QXL_target_Vy, QXL_Chassis->QXL_Chassis_Observe.vy, &QXL_Chassis->QXL_Chassis_Controller.QXL_vy_pid);

	float target_accel_omega  =
			zx_Calc(QXL_Chassis->QXL_Chassis_Controller.QXL_target_Omega, QXL_Chassis->QXL_Chassis_Observe.d_theta, &QXL_Chassis->QXL_Chassis_Controller.QXL_omega_pid);

	//计算目标牵引力及扭矩
	QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Fx = QXL_Chassis->QXL_Chassis_Controller.chassis_m*target_accel_x;
	QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Fy = QXL_Chassis->QXL_Chassis_Controller.chassis_m*target_accel_y;
	QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Tp = QXL_Chassis->QXL_Chassis_Controller.chassis_J*target_accel_omega;

	//计算四个轮子的牵引力
	float Fx=QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Fx;
	float Fy=QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Fy;
	float Tp=QXL_Chassis->QXL_Chassis_Controller.QXL_Chassis_Tp;
	float a_add_b=QXL_Chassis->QXL_Chassis_Observe.a+QXL_Chassis->QXL_Chassis_Observe.b;

	float F1=(sqrt_2f/4.f)*(-Fx + Fy + (Tp/a_add_b) );
	float F2=(sqrt_2f/4.f)*(-Fx - Fy + (Tp/a_add_b) );
	float F3=(sqrt_2f/4.f)*( Fx - Fy + (Tp/a_add_b) );
	float F4=(sqrt_2f/4.f)*( Fx + Fy + (Tp/a_add_b) );

	float _F[4]={F1,F2,F3,F4};

	/*
	 * 此处此处可增加打滑约束  F<=umg/4
	 * 假定车的压力均匀分布
	 * 如果出现某一个  F>umg/4   就会出现打滑的情况
	 * 计算衰减系数lamda=umg*0.25/F
	 * 将四轮的输出F大小均乘lamda限制在umg/4以内
	 */
	float umg=QXL_Chassis->QXL_Chassis_Controller.mju*QXL_Chassis->QXL_Chassis_Controller.chassis_m*g;
	for(int i=0;i<4;i++)
	{
		if( fabsf(_F[i]) > umg/4.f )
		{
			float lamda = (umg/4.f) / fabsf(_F[i]) ;
			   _F[0]   = lamda * _F[0];
			   _F[1]   = lamda * _F[1];
			   _F[2]   = lamda * _F[2];
			   _F[3]   = lamda * _F[3];
		}
	}
	//牵引力重分配
	F1=_F[0];
	F2=_F[1];
	F3=_F[2];
	F4=_F[3];

	//目标控制力矩
	QXL_Chassis->QXL_Chassis_Controller.Wheel_T1 = -QXL_Chassis->QXL_Chassis_Observe.r*F1;
	QXL_Chassis->QXL_Chassis_Controller.Wheel_T2 = -QXL_Chassis->QXL_Chassis_Observe.r*F2;
	QXL_Chassis->QXL_Chassis_Controller.Wheel_T3 = -QXL_Chassis->QXL_Chassis_Observe.r*F3;
	QXL_Chassis->QXL_Chassis_Controller.Wheel_T4 = -QXL_Chassis->QXL_Chassis_Observe.r*F4;

	//(QXL_Chassis->QXL_Chassis_Controller.Wheel_T1 / 5.7 )*(16384/20)

}










