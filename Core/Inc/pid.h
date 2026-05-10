//written by Fred Xiao

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
}pid_str;


extern float pid_3508[12];
extern pid_str pid_3508_speed[12];	
extern pid_str pid_3508_loc[12];	
extern pid_str pid_span;
extern float pid_span_result;


extern int motor_angle1;
extern int motor_angle2;


void PID_Init_All(void);
void PID_Init(pid_str *pid);

float PID_Calc(pid_str *pid,float kp,float ki,float kd,float speed,float feedback_speed);

void position_accumulation(int ID);

void PID_Calc_All(void);



/**
 * @brief 底盘物理参数 (单位: 国际单位制 SI)
 */
typedef struct {
    float wheel_radius; /* 轮半径 r (m) */
    float half_length;  /* 半长 a (m) */
    float half_width;   /* 半宽 b (m) */
    float k_param;      /* k = a + b (m) */
    float mass;         /* 底盘质量 m (kg) */
    float inertia;      /* 转动惯量 J (kg*m^2) */
    float torque_max;   /* 单电机最大输出扭矩 (N*m) */
} Chassis_Params_t;

/**
 * @brief 底盘状态与输入输出数据
 */
typedef struct {
    // [位置环输入] 目标航向角 (由遥控器或导航算法提供)
    float target_theta;  // rad

    // [位置环内部] 里程计估计航向角 (由 wz_obs 积分得到)
    float theta_est;     // rad

    // [速度环输入] 目标控制速度 (由位置环输出或直接设定)
    float target_vx;  // m/s
    float target_vy;  // m/s
    float target_wz;  // rad/s  (由位置环输出)

    // [输入] 轮子当前真实反馈转速 (由电机编码器提供)
    // 编号定义: 1=左前(FL), 2=左后(BL), 3=右后(BR), 4=右前(FR)
    float w1_real;    // rad/s
    float w2_real;    // rad/s
    float w3_real;    // rad/s
    float w4_real;    // rad/s

    // [内部观测] 底盘实际观测速度 (由运动学正解算出)
    float vx_obs;
    float vy_obs;
    float wz_obs;

    // [输出] 电机目标控制扭矩 (前馈 + 反馈)
    float tau_out1;   // N*m
    float tau_out2;   // N*m
    float tau_out3;   // N*m
    float tau_out4;   // N*m
} Chassis_State_t;

/**
 * @brief 核心控制器主结构体
 */
typedef struct {
    Chassis_Params_t params;
    Chassis_State_t  state;

    // 位置环 PID 控制器：用于将航向角误差转化为期望角速度
    pid_str pid_theta;

    // 速度环 PID 控制器：用于将速度误差转化为期望加速度
    pid_str pid_vx;
    pid_str pid_vy;
    pid_str pid_wz;
    
    // 轮端反馈 PID 控制器：用于抵抗打滑和模型误差
    pid_str pid_wheel1;
    pid_str pid_wheel2;
    pid_str pid_wheel3;
    pid_str pid_wheel4;
} Chassis_Force_Controller_t;


/**
 * @brief 底盘力控系统初始化
 * @param chassis 控制器实例指针
 */
void Chassis_Force_Control_Init(Chassis_Force_Controller_t *chassis);

/**
 * @brief 底盘力控系统周期更新函数 (建议运行频率 1kHz)
 * @param chassis 控制器实例指针
 */
void Chassis_Force_Control_Update(Chassis_Force_Controller_t *chassis);

extern Chassis_Force_Controller_t chassis_controller;

void RC_Data_To_Chassis_Target(void);

/* -------------------------------------------------------------------------
     * 物理扭矩 (N·m) 映射到 CAN 总线发送值 (-16384 ~ 16384)
     * 转换系数 = (16384.0f / 20.0f) / 5.7f = 143.7193f
     * -----------------------------------------------------------------------*/
    #define TORQUE_TO_CAN_RATIO 143.7193f

#endif
