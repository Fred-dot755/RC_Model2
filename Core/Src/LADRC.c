#include "ladrc.h"

LADRC_3Order_TypeDef ladrc_speed[2];
LADRC_3Order_TypeDef ladrc_pos[2];
ladrc_ex_TypeDef ladrc_ex[2] ;

int ladrc[2];
float ladrc_control_speed = 0;
float ladrc_control_position = 0;

float target_speed = 0;
float target_position = 0;


void LADRC_Init(LADRC_3Order_TypeDef *ladrc, float wc, float wo, float b0_est, float T, float out_max, float out_min)
{
    // 系统参数
    ladrc->b0 = b0_est;
    ladrc->T = T;
    
    // 控制器参数 - 基于控制器带宽wc（三阶系统）
    ladrc->kp = wc * wc * wc;           // wc^3
    ladrc->kd = 3.0f * wc * wc;         // 3*wc^2  
    ladrc->kdd = 3.0f * wc;             // 3*wc

    
    
    // 观测器参数 - 基于观测器带宽wo（四阶观测器）
    ladrc->beta1 = 4.0f * wo;           // 4*wo
    ladrc->beta2 = 6.0f * wo * wo;      // 6*wo^2
    ladrc->beta3 = 4.0f * wo * wo * wo; // 4*wo^3
    ladrc->beta4 = wo * wo * wo * wo;   // wo^4

    
    // 初始化状态变量
    ladrc->z1 = 0.0f;   // 输出估计（位置/速度）
    ladrc->z2 = 0.0f;   // 一阶导数估计（速度/加速度）
    ladrc->z3 = 0.0f;   // 二阶导数估计（加速度/加加速度）
    ladrc->z4 = 0.0f;   // 总扰动估计
    ladrc->u_prev = 0.0f;
    
    // 设置输出限幅
    ladrc->out_max = out_max;
    ladrc->out_min = out_min;
    
    // 保存带宽值用于调试
    ladrc->wc = wc;
    ladrc->wo = wo;
}



void LADRC_Init_ex(ladrc_ex_TypeDef *ladrc_ex)
{
    ladrc_ex->target_speed = 0;
    ladrc_ex->actual_speed = 0;
    ladrc_ex->target_position = 0;
    ladrc_ex->actual_position = 0;
}

void LADRC_ex_speed(ladrc_ex_TypeDef *ladrc_ex, float targetspeed, float actualspeed)
{
    ladrc_ex->target_speed = targetspeed;
    ladrc_ex->actual_speed = actualspeed;
}

void LADRC_ex_position(ladrc_ex_TypeDef *ladrc_ex, float target_position, float actual_position)
{
    ladrc_ex->target_position = target_position;
    ladrc_ex->actual_position = actual_position;
}


static inline float constrain(float value, float min_val, float max_val) 
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}



/**
  * @brief  LADRC计算函数
  * @param  ladrc: LADRC结构体指针
  * @param  setpoint: 设定值
  * @param  measurement: 测量值
  * @retval 控制量输出
  */
float LADRC_Calc(LADRC_3Order_TypeDef *ladrc, float setpoint, float measurement , int mode)
{
    if(ladrc == NULL) return 0;
    
    float input_scale;
    if(mode == 1)
    {
        input_scale = 1.0f;
    }
    else if(mode == 2)
    {
        input_scale = 0.001f;
    }

    float e, u0, u;
    const float DEAD_ZONE = 0.001f;
    const float Z1_MAX = 10000.0f;
    const float Z2_MAX = 500.0f;
    const float Z3_MAX = 200.0f;
    const float Z4_MAX = 100.0f;
    
    // 1. 线性扩张状态观测器 (LESO) 更新
    e = ladrc->z1 - measurement; // 估计误差
    
    // 死区处理
    if(fabs(e) < DEAD_ZONE) 
    {
        e = 0;
    }
    
    // 使用欧拉法离散化更新状态
    ladrc->z1 += ladrc->T * (ladrc->z2 - ladrc->beta1 * e);
    ladrc->z2 += ladrc->T * (ladrc->z3 - ladrc->beta2 * e);
    ladrc->z3 += ladrc->T * (ladrc->z4 + ladrc->b0 * ladrc->u_prev - ladrc->beta3 * e);
    ladrc->z4 += ladrc->T * (-ladrc->beta4 * e);
    
    // // 状态限幅
    ladrc->z1 = constrain(ladrc->z1, -Z1_MAX, Z1_MAX);
    ladrc->z2 = constrain(ladrc->z2, -Z2_MAX, Z2_MAX);
    ladrc->z3 = constrain(ladrc->z3, -Z3_MAX, Z3_MAX);
    ladrc->z4 = constrain(ladrc->z4, -Z4_MAX, Z4_MAX);
    
    // 2. 扰动补偿与控制量计算
    u0 = ladrc->kp * (setpoint - ladrc->z1)    // 比例项
         - ladrc->kd * ladrc->z2               // 微分项  
         - ladrc->kdd * ladrc->z3;             // 二阶微分项
    
    u = (u0 - ladrc->z4) / ladrc->b0 * input_scale;          // 扰动补偿
    
    // 3. 控制量限幅
    if(u > ladrc->out_max) 
    {
        u = ladrc->out_max;
        // 抗饱和处理
        ladrc->z4 *= 0.99f;
    } else if(u < ladrc->out_min) 
    {
        u = ladrc->out_min;
        // 抗饱和处理
        ladrc->z4 *= 0.99f;
    }
    
    // 保存当前控制量，用于下一次观测器更新
    ladrc->u_prev = u;
    
    return u;
}
