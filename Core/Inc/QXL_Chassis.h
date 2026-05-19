/*
 * QXL_Chassis.h
 *
 *  Created on: May 3, 2026
 *      Author: zsh
 */

#ifndef QXL_CHASSIS_INC_QXL_CHASSIS_H_
#define QXL_CHASSIS_INC_QXL_CHASSIS_H_

/*				^y
 * 			    |
 * 		4		|    1
 * 				|
 * ------------------------->x
 * 		3		|    2
 * 			    |
 * 			    |
 * 		V1,v2,v3,v4分别为四个轮子的线速度，按照象限排序,vx,vy,d_theta为底盘的线速度和角速度
 * 		底盘中心为坐标原点，第一象限轮中心坐标(a,b);
 *
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "zx_pid.h"

#ifdef __cplusplus
}
#endif

typedef struct QXL_chassis_observe_s{
	float a,b;
	float r;

	//状态
	float vx;
	float vy;
	float d_theta;

	//实际观测
	float v[4];//v1,v2,v3,v4
	float gyro_yaw;
	float N;

}QXL_chassis_observe_t;

typedef struct QXL_chassis_controller_s{

	float mju;            //动摩擦因数
	float chassis_m;      //车质量    22.5
	float chassis_J;      //底盘转动惯量0.379

	zxPID QXL_vx_pid;     //p控制
	zxPID QXL_vy_pid;     //p控制
	zxPID QXL_omega_pid;  //pi控制

	float QXL_target_Vx;   //底盘目标vx
	float QXL_target_Vy;   //底盘目标vy
	float QXL_target_Omega;//底盘目标角速度

	float QXL_Chassis_Fx; //底盘目标牵引力x分量
	float QXL_Chassis_Fy; //底盘目标牵引力y分量
	float QXL_Chassis_Tp; //底盘目标转矩  T

	float Wheel_T1;       //轮电机1  扭矩
	float Wheel_T2;       //轮电机2  扭矩
	float Wheel_T3;       //轮电机3  扭矩
	float Wheel_T4;       //轮电机4  扭矩

}QXL_chassis_controller_t;

typedef struct QXL_chassis_s{
	QXL_chassis_observe_t QXL_Chassis_Observe;
	QXL_chassis_controller_t QXL_Chassis_Controller;

}QXL_chassis_t;

extern QXL_chassis_t QXL_Chassis_Controller;


//C函数接口
#ifdef __cplusplus
extern "C" {
#endif

void QXL_Chassis_Init(QXL_chassis_t* QXL_Chassis);
void QXL_chassis_observe_update(float v[4],float gyro_yaw, QXL_chassis_t* QXL_Chassis,float dt);

void QXL_chassis_controller_init(float vx_kp,float vy_kp,float omega_kp,float omega_ki,float mju,float m,float J,QXL_chassis_t* QXL_Chassis);
void QXL_chassis_controller_update(float _target_vx,float _target_vy,float _target_omega,QXL_chassis_t* QXL_Chassis);

#ifdef __cplusplus
}
#endif



#ifdef __cplusplus
#include "matrix.h"

#define cos_f arm_cos_f32
#define sin_f arm_sin_f32

const float sqrt_2f=1.414213562373f;
/*------------------------------工具函数----------------------------------------------*/
static void QXL_chassis_kalman_F_update(float d_theta,float dt,float* F_data){

	float F[3*3]={
		cos_f(d_theta*dt) , sin_f(d_theta*dt) , 0 ,
		-sin_f(d_theta*dt), cos_f(d_theta*dt) , 0 ,
		0                 , 0                 , 1
	};

	memcpy(F_data,F,9*4);
}

static void QXL_chassis_kalman_H_update(float a,float b,float* H_data){

	float H[5*3]={
		-(1.f/sqrt_2f) ,  (1.f/sqrt_2f) , (a+b)/sqrt_2f,
		-(1.f/sqrt_2f) , -(1.f/sqrt_2f) , (a+b)/sqrt_2f,
		 (1.f/sqrt_2f) , -(1.f/sqrt_2f) , (a+b)/sqrt_2f,
		 (1.f/sqrt_2f) ,  (1.f/sqrt_2f) , (a+b)/sqrt_2f,
		 0              ,       0       ,      1
	};

	memcpy(H_data,H,15*4);

}
/*------------------------------------底盘类----------------------------------------------------*/
class QXL_chassis{
public:
	//构造函数
	QXL_chassis(const float _a,const float _b,float* Q_data,float* R_data,float* P_data):
												a(_a)        , b(_b)       , mat_x(),
												mat_z()      , mat_F()     , mat_H(),
												mat_P(P_data) , mat_K()	   , mat_Q(Q_data),
												mat_R(R_data), z_residual(), x_residual(),
												S_k()		 , _S_k()
	{
	}
	~QXL_chassis(){};

	void Observe_kalman_update(QXL_chassis_observe_t* observe,float dt){
		this->N=observe->N;

		this->a=observe->a;
		this->b=observe->b;

		//更新观测
		this->mat_z[0][0]=observe->v[0];
		this->mat_z[1][0]=observe->v[1];
		this->mat_z[2][0]=observe->v[2];
		this->mat_z[3][0]=observe->v[3];
		this->mat_z[4][0]=observe->gyro_yaw;

		//更新状态转移矩阵F
		QXL_chassis_kalman_F_update(this->mat_x[2][0],dt,(float*)&this->mat_F[0][0]);
		//更新观测矩阵H
		QXL_chassis_kalman_H_update(this->a,this->b,(float*)&this->mat_H[0][0]);

		//先验状态估计
		Matrixf<3,1> mat_x_prior = this->mat_F * this->mat_x ;
		//先验协方差估计
		Matrixf<3,3> mat_P_prior = this->mat_F * this->mat_P * this->mat_F.trans() + this->mat_Q ;
		//计算卡尔曼增益
		Matrixf<5,5> mat_mid_inv = matrixf::inv(this->mat_H * mat_P_prior * this->mat_H.trans() + this->mat_R);
		this->mat_K = mat_P_prior * this->mat_H.trans() * mat_mid_inv ;
		//后验状态估计
		this->mat_x = mat_x_prior + this->mat_K * (this->mat_z - this->mat_H * mat_x_prior) ;
		//后验协方差估计
		this->mat_P = (matrixf::eye<3,3>() - this->mat_K * this->mat_H) * mat_P_prior ;

		//估计观测残差，观测协方差

		observe->vx      = this->mat_x[0][0];
		observe->vy      = this->mat_x[1][0];
		observe->d_theta = this->mat_x[2][0];

	}


private:

	float a,b;


	Matrixf<3,1> mat_x;      //状态
	Matrixf<5,1> mat_z;      //观测

	Matrixf<3,3> mat_F;      //状态转移矩阵
	Matrixf<5,3> mat_H;      //观测矩阵
	Matrixf<3,3> mat_P;      //状态协方差矩阵
	Matrixf<3,5> mat_K;      //卡尔曼增益矩阵
	Matrixf<3,3> mat_Q;      //过程噪声协方差矩阵
	Matrixf<5,5> mat_R;      //观测噪声协方差矩阵

	Matrixf<5,1> z_residual; //观测残差
	Matrixf<3,1> x_residual; //状态残差

	Matrixf<5,5> S_k;		 //本次估计残差协方差矩阵
	Matrixf<5,5> _S_k;		 //估计残差协方差矩阵

	float N;                 //滑动窗口大小   20-100



};

#endif







#endif /* QXL_CHASSIS_INC_QXL_CHASSIS_H_ */
