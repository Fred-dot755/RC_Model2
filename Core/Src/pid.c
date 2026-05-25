//written by Fred Xiao

#include "pid.h"

float pid_3508[12];
pid_str pid_3508_speed[12];		//速度PID
pid_str pid_3508_loc[12];			//位置PID

pid_str pid_span;
float pid_span_result;

int motor_angle1;
int motor_angle2;


/**
 * @brief 浮点数限位函数
 */
static inline float Clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}


void PID_Init_All(void)
{
    for(int i=0;i<12;i++)
    {
        PID_Init(&pid_3508_speed[i]);
        PID_Init(&pid_3508_loc[i]);
    }
	PID_Init(&pid_span);
}

void PID_Init(pid_str *pid)
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


float PID_Calc(pid_str *pid,float kp,float ki,float kd,float speed,float feedback_speed)
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


void PID_Calc_All(void)
{
	pid_3508[0] =  Clamp(PID_Calc(&pid_3508_speed[0], 10.0f, 0.025f, 0.0f, result.motor1.out, motor[0].NowSpeed),            -16384.0f, 16384.0f);
    pid_3508[1] =  Clamp(-PID_Calc(&pid_3508_speed[1], 10.0f, 0.025f, 0.0f, result.motor2.out, -motor[1].NowSpeed),           -16384.0f, 16384.0f);
    pid_3508[2] =  Clamp(-PID_Calc(&pid_3508_speed[2], 10.0f, 0.025f, 0.0f, result.motor3.out, -motor[2].NowSpeed),           -16384.0f, 16384.0f);
    pid_3508[3] =  Clamp(PID_Calc(&pid_3508_speed[3], 10.0f, 0.025f, 0.0f, result.motor4.out, motor[3].NowSpeed),            -16384.0f, 16384.0f);
    pid_3508[4] =  Clamp(-PID_Calc(&pid_3508_speed[4], 3.0f, 0.015f, 0.0f, result.motor5.out, -motor[4].NowSpeed),           -16384.0f, 16384.0f);
    pid_3508[5] =  Clamp(PID_Calc(&pid_3508_speed[5], 3.0f, 0.015f, 0.0f, result.motor6.out, motor[5].NowSpeed),            -16384.0f, 16384.0f);
	pid_3508[6] =  Clamp(PID_Calc(&pid_3508_speed[6], 3.0f, 0.015f, 0.0f, result.motor7.out, motor[6].NowSpeed),            -16384.0f, 16384.0f);
	pid_3508[7] =  Clamp(-PID_Calc(&pid_3508_speed[7], 3.0f, 0.015f, 0.0f, result.motor8.out, -motor[7].NowSpeed),           -16384.0f, 16384.0f);

	pid_3508[8] =  Clamp(PID_Calc(&pid_3508_loc[8], 1.40, 0.10, 0.70, PID_Calc(&pid_3508_speed[8], 0.55, 0, 0.20, motor[8].ExpectAngle, motor[8].accumulated_distance), motor[8].NowSpeed),   -2000.0f, 2000.0f);
	pid_3508[9] =  Clamp(PID_Calc(&pid_3508_loc[9], 1.40, 0.10, 0.70, PID_Calc(&pid_3508_speed[9], 0.55, 0, 0.20, motor[9].ExpectAngle, motor[9].accumulated_distance), motor[9].NowSpeed),   -2000.0f, 2000.0f);

	// pid_3508[10] = Clamp(PID_Calc(&pid_3508_loc[10], 1.40, 0.10, 0.70, PID_Calc(&pid_3508_speed[10], 0.55, 0, 0.20, motor[10].ExpectAngle, motor[10].accumulated_distance), motor[10].NowSpeed), -16384.0f, 16384.0f);
	// pid_3508[11] = Clamp(PID_Calc(&pid_3508_loc[11], 1.40, 0.005, 3.50, PID_Calc(&pid_3508_speed[11], 0.55, 0, 0.20, motor[11].ExpectAngle, motor[11].accumulated_distance), motor[11].NowSpeed), -16384.0f, 16384.0f);

    //10 472552
}





Chassis_Force_Controller_t chassis_controller;
#define SQRT2 1.4142135f




void Chassis_Force_Control_Init(Chassis_Force_Controller_t *chassis) {
    // 初始化底盘物理参数
    // chassis->params.wheel_radius = 0.076f;      /* 麦克纳姆轮半径 76mm */
    // chassis->params.half_length  = 0.155f;      /* 半长 a */
    // chassis->params.half_width   = 0.177f;      /* 半宽 b */
    // chassis->params.k_param      = 0.332f;      /* k = a + b */
    // chassis->params.mass         = 25.0f;       /* 底盘质量 */
    // chassis->params.inertia      = 0.461f;      /* 转动惯量 J */
    // chassis->params.torque_max   = 80.0f;        /* 最大扭矩 */

    chassis->params.wheel_radius = 0.07275f;      /* 全向轮半径 m */
    chassis->params.half_length  = 0.155f;      /* 半长 a */
    chassis->params.half_width   = 0.177f;      /* 半宽 b */
    chassis->params.k_param      = 0.235f;      /* k = (a + b)/√2 */
    chassis->params.mass         = 25.0f;       /* 底盘质量 */
    chassis->params.inertia      = 0.461f;      /* 转动惯量 J */
    chassis->params.torque_max   = 80.0f;        /* 最大扭矩 */

    // 初始化航向角位置环 PID 参数
    chassis->pid_theta.Kp = 10.0f;  chassis->pid_theta.Ki = 0.1f; chassis->pid_theta.Kd = 1.0f;
    chassis->pid_theta.I_Max = 1.0f; chassis->pid_theta.V_Max = 6.0f;  /* 输出限幅 rad/s */

    // 初始化目标加速度 PID 参数
    // vx
    chassis->pid_vx.Kp = 20.0f;  chassis->pid_vx.Ki = 0.1f; chassis->pid_vx.Kd = 0.0f;
    chassis->pid_vx.I_Max = 10.0f; chassis->pid_vx.V_Max = 40.0f;
    // vy
    chassis->pid_vy.Kp = 20.0f;  chassis->pid_vy.Ki = 0.1f; chassis->pid_vy.Kd = 0.0f;
    chassis->pid_vy.I_Max = 10.0f; chassis->pid_vy.V_Max = 40.0f;
    // wz
    chassis->pid_wz.Kp = 10.0f;  chassis->pid_wz.Ki = 0.5f; chassis->pid_wz.Kd = 1.0f;
    chassis->pid_wz.I_Max = 0.0f; chassis->pid_wz.V_Max = 10.0f;

    // 初始化位置环状态
    chassis->state.target_theta = 0.0f;
    chassis->state.theta_est    = 0.0f;

    chassis->pid_wheel1.Kp = 5.0f; chassis->pid_wheel1.Ki = 0.025f; chassis->pid_wheel1.Kd = 0.0f;
    chassis->pid_wheel1.I_Max = 5.0f; chassis->pid_wheel1.V_Max = 80;
    
    chassis->pid_wheel2.Kp = 5.0f; chassis->pid_wheel2.Ki = 0.025f; chassis->pid_wheel2.Kd = 0.0f;
    chassis->pid_wheel2.I_Max = 5.0f; chassis->pid_wheel2.V_Max = 80;
    
    chassis->pid_wheel3.Kp = 5.0f; chassis->pid_wheel3.Ki = 0.025f; chassis->pid_wheel3.Kd = 0.0f;
    chassis->pid_wheel3.I_Max = 5.0f; chassis->pid_wheel3.V_Max = 80;
    
    chassis->pid_wheel4.Kp = 5.0f; chassis->pid_wheel4.Ki = 0.025f; chassis->pid_wheel4.Kd = 0.0f;
    chassis->pid_wheel4.I_Max = 5.0f; chassis->pid_wheel4.V_Max = 80;
}

//全向轮解算
// void Chassis_Force_Control_Update(Chassis_Force_Controller_t *chassis) {
//     float r = chassis->params.wheel_radius;
//     float k = chassis->params.k_param;
//     float m = chassis->params.mass;
//     float J = chassis->params.inertia;
    
//     chassis->state.w1_real = (float)motor[0].NowSpeed * RPM_TO_RADS;
//     chassis->state.w2_real = (float)-motor[1].NowSpeed * RPM_TO_RADS;
//     chassis->state.w3_real = (float)-motor[2].NowSpeed * RPM_TO_RADS;
//     chassis->state.w4_real = (float)motor[3].NowSpeed * RPM_TO_RADS;

//     float w1 =  chassis->state.w1_real;
//     float w2 =  chassis->state.w2_real;
//     float w3 =  chassis->state.w3_real;
//     float w4 =  chassis->state.w4_real;


//     float sqrt2_r_4 = (SQRT2 * r) / 4.0f;
//     float r_4k = r / (4.0f * k);

//     chassis->state.vx_obs =  sqrt2_r_4 * (w1 - w2 + w3 - w4);
//     chassis->state.vy_obs =  sqrt2_r_4 * (w1 + w2 + w3 + w4);
//     chassis->state.wz_obs =  r_4k * (w1 - w2 - w3 + w4);


//     float theta_feedback = ops.HIPNUCAngleZ * PI / 180.0f;
    
//     float theta_err = chassis->state.target_theta - theta_feedback;
//     while (theta_err >  PI) theta_err -= 2.0f * PI;
//     while (theta_err < -PI) theta_err += 2.0f * PI;
    
//     chassis->state.target_wz = -PID_Calc(&chassis->pid_theta,
//                                           chassis->pid_theta.Kp, chassis->pid_theta.Ki, chassis->pid_theta.Kd,
//                                           theta_err, 0.0f);

//     float ax = PID_Calc(&chassis->pid_vx, chassis->pid_vx.Kp, chassis->pid_vx.Ki, chassis->pid_vx.Kd,
//                         chassis->state.target_vx, chassis->state.vx_obs);
    
//     float ay = PID_Calc(&chassis->pid_vy, chassis->pid_vy.Kp, chassis->pid_vy.Ki, chassis->pid_vy.Kd,
//                         chassis->state.target_vy, chassis->state.vy_obs);
    
//     float alpha = PID_Calc(&chassis->pid_wz, chassis->pid_wz.Kp, chassis->pid_wz.Ki, chassis->pid_wz.Kd,
//                            chassis->state.target_wz, chassis->state.wz_obs);

//     float sqrt2_r_4_m = sqrt2_r_4 * m;
//     float tau_wz_ff = r_4k * J * alpha;

//     float tau_ff1 = sqrt2_r_4_m * ( ax + ay) + tau_wz_ff;
//     float tau_ff2 = sqrt2_r_4_m * (-ax + ay) - tau_wz_ff;
//     float tau_ff3 = sqrt2_r_4_m * ( ax + ay) - tau_wz_ff;
//     float tau_ff4 = sqrt2_r_4_m * (-ax + ay) + tau_wz_ff;

//     float sqrt2_2r = SQRT2 / (2.0f * r);
//     float k_r = k / r;

//     float w1_exp = sqrt2_2r * ( chassis->state.target_vx + chassis->state.target_vy) + k_r * chassis->state.target_wz;
//     float w2_exp = sqrt2_2r * (-chassis->state.target_vx + chassis->state.target_vy) - k_r * chassis->state.target_wz;
//     float w3_exp = sqrt2_2r * ( chassis->state.target_vx + chassis->state.target_vy) - k_r * chassis->state.target_wz;
//     float w4_exp = sqrt2_2r * (-chassis->state.target_vx + chassis->state.target_vy) + k_r * chassis->state.target_wz;

//     float tau_pid1 = PID_Calc(&chassis->pid_wheel1, chassis->pid_wheel1.Kp, chassis->pid_wheel1.Ki, chassis->pid_wheel1.Kd,
//                               w1_exp, w1);
//     float tau_pid2 = PID_Calc(&chassis->pid_wheel2, chassis->pid_wheel2.Kp, chassis->pid_wheel2.Ki, chassis->pid_wheel2.Kd,
//                               w2_exp, w2);
//     float tau_pid3 = PID_Calc(&chassis->pid_wheel3, chassis->pid_wheel3.Kp, chassis->pid_wheel3.Ki, chassis->pid_wheel3.Kd,
//                               w3_exp, w3);
//     float tau_pid4 = PID_Calc(&chassis->pid_wheel4, chassis->pid_wheel4.Kp, chassis->pid_wheel4.Ki, chassis->pid_wheel4.Kd,
//                               w4_exp, w4);
    
//     float t_max = chassis->params.torque_max;
    
//     chassis->state.tau_out1 =  Clamp(tau_ff1 + tau_pid1, -t_max, t_max);
//     chassis->state.tau_out2 = -Clamp(tau_ff2 + tau_pid2, -t_max, t_max);
//     chassis->state.tau_out3 = -Clamp(tau_ff3 + tau_pid3, -t_max, t_max);
//     chassis->state.tau_out4 =  Clamp(tau_ff4 + tau_pid4, -t_max, t_max);

//     pid_3508[0] = Clamp((int)(chassis->state.tau_out1 * TORQUE_TO_CAN_RATIO), -16384, 16384);
//     pid_3508[1] = Clamp((int)(chassis->state.tau_out2 * TORQUE_TO_CAN_RATIO), -16384, 16384);
//     pid_3508[2] = Clamp((int)(chassis->state.tau_out3 * TORQUE_TO_CAN_RATIO), -16384, 16384);
//     pid_3508[3] = Clamp((int)(chassis->state.tau_out4 * TORQUE_TO_CAN_RATIO), -16384, 16384);
// }

//麦克纳姆轮解算
void Chassis_Force_Control_Update(Chassis_Force_Controller_t *chassis) {
    float r = chassis->params.wheel_radius;
    float k = chassis->params.k_param;
    float m = chassis->params.mass;
    float J = chassis->params.inertia;
    
    chassis->state.w1_real = (float)motor[0].NowSpeed * RPM_TO_RADS;
    chassis->state.w2_real = (float)-motor[1].NowSpeed * RPM_TO_RADS;
    chassis->state.w3_real = (float)-motor[2].NowSpeed * RPM_TO_RADS;
    chassis->state.w4_real = (float)motor[3].NowSpeed * RPM_TO_RADS;

    float w1 =  chassis->state.w1_real;
    float w2 =  chassis->state.w2_real;
    float w3 =  chassis->state.w3_real;
    float w4 =  chassis->state.w4_real;

    float sqrt2_r_4 = (SQRT2 * r) / 4.0f;


    chassis->state.vx_obs =  sqrt2_r_4 * (w1 + w2 + w3 + w4);
    chassis->state.vy_obs =  sqrt2_r_4 * (-w1 + w2 - w3 + w4);
    chassis->state.wz_obs =  sqrt2_r_4 * (1.0f / k) * (w1 - w2 - w3 + w4);


    float theta_feedback = ops.HIPNUCAngleZ * PI / 180.0f;
    
    float theta_err = chassis->state.target_theta - theta_feedback;
    while (theta_err >  PI) theta_err -= 2.0f * PI;
    while (theta_err < -PI) theta_err += 2.0f * PI;
    
    chassis->state.target_wz = -PID_Calc(&chassis->pid_theta,
                                          chassis->pid_theta.Kp, chassis->pid_theta.Ki, chassis->pid_theta.Kd,
                                          theta_err, 0.0f);


    float ax = PID_Calc(&chassis->pid_vx, chassis->pid_vx.Kp, chassis->pid_vx.Ki, chassis->pid_vx.Kd,
                        chassis->state.target_vx, chassis->state.vx_obs);
    
    float ay = PID_Calc(&chassis->pid_vy, chassis->pid_vy.Kp, chassis->pid_vy.Ki, chassis->pid_vy.Kd,
                        chassis->state.target_vy, chassis->state.vy_obs);
    
    float alpha = PID_Calc(&chassis->pid_wz, chassis->pid_wz.Kp, chassis->pid_wz.Ki, chassis->pid_wz.Kd,
                           chassis->state.target_wz, chassis->state.wz_obs);


    float p1_dot_a = -ax + ay;
    float p2_dot_a = -ax - ay;
    float p3_dot_a =  ax - ay;
    float p4_dot_a =  ax + ay;

    float torque_theta = (J / k) * alpha;

    float tau_ff1 = -sqrt2_r_4 * (m * p1_dot_a + torque_theta);
    float tau_ff2 = -sqrt2_r_4 * (m * p2_dot_a + torque_theta);
    float tau_ff3 = -sqrt2_r_4 * (m * p3_dot_a + torque_theta);
    float tau_ff4 = -sqrt2_r_4 * (m * p4_dot_a + torque_theta);


    float sqrt2_invR = SQRT2 / r;
    float w1_exp = sqrt2_invR * ( chassis->state.target_vx - chassis->state.target_vy + k * chassis->state.target_wz);
    float w2_exp = sqrt2_invR * ( chassis->state.target_vx + chassis->state.target_vy - k * chassis->state.target_wz);
    float w3_exp = sqrt2_invR * ( chassis->state.target_vx - chassis->state.target_vy - k * chassis->state.target_wz);
    float w4_exp = sqrt2_invR * ( chassis->state.target_vx + chassis->state.target_vy + k * chassis->state.target_wz);

    float tau_pid1 = PID_Calc(&chassis->pid_wheel1, chassis->pid_wheel1.Kp, chassis->pid_wheel1.Ki, chassis->pid_wheel1.Kd,
                              w1_exp, w1);
    float tau_pid2 = PID_Calc(&chassis->pid_wheel2, chassis->pid_wheel2.Kp, chassis->pid_wheel2.Ki, chassis->pid_wheel2.Kd,
                              w2_exp, w2);
    float tau_pid3 = PID_Calc(&chassis->pid_wheel3, chassis->pid_wheel3.Kp, chassis->pid_wheel3.Ki, chassis->pid_wheel3.Kd,
                              w3_exp, w3);
    float tau_pid4 = PID_Calc(&chassis->pid_wheel4, chassis->pid_wheel4.Kp, chassis->pid_wheel4.Ki, chassis->pid_wheel4.Kd,
                              w4_exp, w4);

    float t_max = chassis->params.torque_max;
    
    chassis->state.tau_out1 =  Clamp(tau_ff1 + tau_pid1, -t_max, t_max);
    chassis->state.tau_out2 = -Clamp(tau_ff2 + tau_pid2, -t_max, t_max);
    chassis->state.tau_out3 = -Clamp(tau_ff3 + tau_pid3, -t_max, t_max);
    chassis->state.tau_out4 =  Clamp(tau_ff4 + tau_pid4, -t_max, t_max);


    pid_3508[0] = (int)(chassis->state.tau_out1 * TORQUE_TO_CAN_RATIO);
    pid_3508[1] = (int)(chassis->state.tau_out2 * TORQUE_TO_CAN_RATIO);
    pid_3508[2] = (int)(chassis->state.tau_out3 * TORQUE_TO_CAN_RATIO);
    pid_3508[3] = (int)(chassis->state.tau_out4 * TORQUE_TO_CAN_RATIO);

}



#define MAX_CHASSIS_SPEED  1.0f
#define MAX_CHASSIS_WZ     3.0f

void RC_Data_To_Chassis_Target(void) {
    
    float target_speed = R2_Extern.speed;
    float target_angle = R2_Extern.angle;
    // float target_speed = rc_data.distance * 3;
    // float target_angle = rc_data.angle;
    
    float angle_rad = target_angle * PI / 180.0f;
    
    float vx = target_speed * sinf(angle_rad);
    float vy = -target_speed * cosf(angle_rad);
    
    float speed_mag = sqrtf(vx * vx + vy * vy);
    if (speed_mag > MAX_CHASSIS_SPEED) {
        float scale = MAX_CHASSIS_SPEED / speed_mag;
        vx *= scale;
        vy *= scale;
    }
    chassis_controller.state.target_vx = vx;
    chassis_controller.state.target_vy = vy;

    chassis_controller.state.target_theta = R2_Extern.angle_balance * PI / 180.0f;
}




