#ifndef __LADRC_H
#define __LADRC_H

#include "main.h"

#define wc_pos 30.0f
#define wo_pos 100.0f

extern int ladrc[2];


typedef struct {
    // 控制器增益
    float kp, kd, kdd;          // 状态反馈增益
    
    // 观测器增益  
    float beta1, beta2, beta3, beta4;  // 4阶观测器（3阶系统+扩张状态）
    
    // 系统参数
    float b0;                   // 控制增益估计值
    float T;                    // 控制周期
    
    // 观测器状态
    float z1, z2, z3, z4;       // z1:位置, z2:速度, z3:加速度, z4:总扰动
    float u_prev;               // 上一次控制量
    
    // 限幅
    float out_max, out_min;
    
    // 调试信息
    float wc, wo;               // 保存带宽值
} LADRC_3Order_TypeDef;


typedef struct 
{
    float target_speed;
    float actual_speed;
    float target_position;
    float actual_position;
}ladrc_ex_TypeDef;

extern LADRC_3Order_TypeDef ladrc_speed[2];
extern LADRC_3Order_TypeDef ladrc_pos[2];
extern ladrc_ex_TypeDef ladrc_ex[2] ;

extern float ladrc_control_speed;
extern float ladrc_control_position;

extern float target_speed;
extern float target_position;

// 函数声明
void LADRC_Init(LADRC_3Order_TypeDef *ladrc, float wc, float wo, float b0_est, float T, float out_max, float out_min);
float LADRC_Calc(LADRC_3Order_TypeDef *ladrc, float setpoint, float measurement , int mode);
void LADRC_Init_ex(ladrc_ex_TypeDef *ladrc_ex);
void LADRC_ex_speed(ladrc_ex_TypeDef *ladrc_ex, float targetspeed, float actualspeed);
void LADRC_ex_position(ladrc_ex_TypeDef *ladrc_ex, float target_position, float actual_position);

static inline float constrain(float value, float min_val, float max_val) ;


#endif

